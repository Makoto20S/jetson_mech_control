// Only newly created PTYs and temporary files. No actual TTY or CAN admission.
#define main duplex_cli_entry
#include "main.cpp"
#undef main
#include "mech_bringup/pass_through_init.hpp"
#include <atomic>
#include <deque>
#include <exception>

namespace { std::deque<int> write_actions; }
extern "C" ssize_t __real_write(int,const void*,size_t);
extern "C" ssize_t __wrap_write(int fd,const void* bytes,size_t size) {
  if(fd<=2||write_actions.empty())return __real_write(fd,bytes,size);
  const auto action=write_actions.front();write_actions.pop_front();
  if(action<0){errno=-action;return -1;}
  return action==0?0:__real_write(fd,bytes,std::min(size,static_cast<std::size_t>(action)));
}
namespace {
using namespace duplex;
using mech::mech_bringup::PosixCdcSerialPort;
unsigned checks=0;
void check(bool value,const char* error){++checks;if(!value)throw std::runtime_error(error);}
template<class F>void reject(F action){bool caught=false;try{action();}catch(const std::exception&){caught=true;}check(caught,"expected reject");}
Bytes init(){const auto& bytes=mech::mech_bringup::kPassThroughInitFrame;return Bytes(bytes.begin(),bytes.end());}
class Pty {
 public:
  int master{-1};std::string slave;
  Pty(){master=::posix_openpt(O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);check(master>=0,"PTY open");check(::grantpt(master)==0&&::unlockpt(master)==0,"PTY unlock");char path[256]{};check(::ptsname_r(master,path,sizeof(path))==0,"PTY name");slave=path;}
  ~Pty(){if(master>=0)::close(master);}
  Bytes drain(std::size_t wanted){Bytes out;std::array<std::uint8_t,1024> bytes{};auto deadline=Clock::now()+std::chrono::seconds(1);while(out.size()<wanted&&Clock::now()<deadline){const auto n=::read(master,bytes.data(),std::min(bytes.size(),wanted-out.size()));if(n>0)out.insert(out.end(),bytes.begin(),bytes.begin()+n);else if(n<0&&errno!=EAGAIN&&errno!=EINTR)break;else std::this_thread::yield();}return out;}
};
struct Temp {
  fs::path path;
  Temp(){char name[]="/tmp/duplex-offline-test-XXXXXX";const auto made=::mkdtemp(name);check(made!=nullptr,"temp");path=made;}
  ~Temp(){std::error_code error;fs::remove_all(path,error);}
};
std::string read_text(const fs::path& path){std::ifstream stream(path);check(stream.is_open(),"read temp metadata");std::ostringstream out;out<<stream.rdbuf();return out.str();}
void relay_happy(){
  Pty a,b;PosixCdcSerialPort pa(a.slave),pb(b.slave);check(pa.open()&&pb.open(),"owned PTY ports open");
  Config config;config.seconds=1;config.hz=10;config.feedback_hz=2;
  Capture ca(4*1024*1024),cb(4*1024*1024);
  std::atomic<bool> ready{false},done{false};std::exception_ptr peer_error;
  std::array<Bytes,2> pending;std::array<unsigned,2> initializes{},frames{};
  std::thread peer([&]{try{
    const auto deadline=Clock::now()+std::chrono::seconds(7);
    while(!done&&Clock::now()<deadline){
      for(unsigned port=0;port<2;++port){
        std::array<std::uint8_t,1024> bytes{};
        const auto n=::read(port?b.master:a.master,bytes.data(),bytes.size());
        if(n>0)pending[port].insert(pending[port].end(),bytes.begin(),bytes.begin()+n);
        else if(n<0&&errno!=EAGAIN&&errno!=EINTR&&errno!=EIO)throw std::runtime_error("relay read");
        auto& buffer=pending[port];
        while(buffer.size()>=7){
          const auto size=static_cast<std::size_t>(buffer[2]|(static_cast<unsigned>(buffer[3])<<8))+7;
          if(buffer.size()<size)break;
          const Bytes packet(buffer.begin(),buffer.begin()+size);
          if(packet==init()){if(++initializes[port]!=1)throw std::runtime_error("multiple init");}
          else{
            if(!ready||size!=21)throw std::runtime_error("CAN before quiet/readiness or joined write");
            const auto expected=packets(port==1);if(packet!=expected[frames[port]%2])throw std::runtime_error("wrong production packet");
            ++frames[port];
            if(__real_write(port?a.master:b.master,packet.data(),packet.size())!=static_cast<ssize_t>(packet.size()))throw std::runtime_error("bounded relay write");
          }
          buffer.erase(buffer.begin(),buffer.begin()+size);
        }
      }
      std::this_thread::sleep_for(std::chrono::microseconds(100));
    }
  }catch(...){peer_error=std::current_exception();}});
  const auto result=run(pa,pb,config,ca,cb,[]{return false;},[&]{ready=true;});
  done=true;peer.join();if(peer_error)std::rethrow_exception(peer_error);
  check(result.complete()&&result.content_match,"full duplex relay matches");
  check(initializes[0]==1&&initializes[1]==1&&frames[0]==20&&frames[1]==4,"each initialization and planned count");
  check(result.a.tx_frames==20&&result.b.tx_frames==4&&result.a.rx_frames==4&&result.b.rx_frames==20,"both ports serviced");
  check(!pa.is_open()&&!pb.is_open()&&result.a.closed&&result.b.closed,"both close");
  PosixCdcSerialPort reopen_a(a.slave),reopen_b(b.slave);check(reopen_a.open()&&reopen_b.open(),"both locks released");
}
void partial_first_init(){
  Pty a,b;PosixCdcSerialPort pa(a.slave),pb(b.slave);check(pa.open()&&pb.open(),"partial ports open");
  Config config;config.seconds=1;Capture ca(8192),cb(8192);unsigned ready=0;
  write_actions={5,-EAGAIN};const auto result=run(pa,pb,config,ca,cb,[]{return false;},[&]{++ready;});write_actions.clear();
  check(!result.complete()&&ready==0,"partial init aborts readiness");
  check(!pa.is_open()&&!pb.is_open(),"partial closes both");const auto expected=init();
  check(a.drain(5)==Bytes(expected.begin(),expected.begin()+5),"only partial init bytes");
  check(b.drain(1).empty(),"other port no later init or CAN");
}
void admission_and_supervisor(){
  Temp temp;Pty p;Identity id;struct stat st{};check(::stat(p.slave.c_str(),&st)==0,"PTY identity");id.device=st.st_rdev;id.usb="/fake/usb/1-3.2";
  fs::create_directories(temp.path/"task"/"fd");inspect_fds(temp.path/"task",id);
  fs::create_symlink(p.slave,temp.path/"task"/"fd"/"8");reject([&]{inspect_fds(temp.path/"task",id);});
  auto sibling=id;sibling.device=makedev(999,1);sibling.usb=(temp.path/"usb"/"1-3.2").string();
  fs::create_directories(fs::path(sibling.usb)/"1-3.2:1.1");const auto sysroot=temp.path/"sys-char";
  const auto dev=sysroot/(std::to_string(major(st.st_rdev))+":"+std::to_string(minor(st.st_rdev)));fs::create_directories(dev);fs::create_symlink(fs::path(sibling.usb)/"1-3.2:1.1",dev/"device");
  reject([&]{inspect_fds(temp.path/"task",sibling,sysroot);});reject([&]{identify(p.slave);});
  write_new(temp.path/"new","original");reject([&]{write_new(temp.path/"new","replacement");});check(read_text(temp.path/"new")=="original","no overwrite");
  for(unsigned scenario=0;scenario<5;++scenario){
    const auto output=temp.path/std::to_string(scenario);fs::create_directory(output);stopped=0;const auto child=::fork();check(child>=0,"worker fork");
    if(child==0){if(scenario==3){::signal(SIGTERM,SIG_IGN);for(;;)::pause();}try{
      write_new(output/"plan.json","{}\n");write_new(output/"summary.json","{}\n");write_new(output/"ready.json","{}\n");write_new(output/"capture-a.bin","");if(scenario!=1)write_new(output/"capture-b.bin","");
    }catch(...){::_exit(3);}::_exit(scenario==2?1:(scenario==4?4:0));}
    const auto result=supervise(child,output,"{}",std::chrono::milliseconds(scenario==3?150:1000));check(result==(scenario==0?0:(scenario==4?4:1)),"supervisor result");
    int status=0;errno=0;check(::waitpid(child,&status,WNOHANG)==-1&&errno==ECHILD,"worker reaped");
    check(read_text(output/"supervisor.json").find("\"ports_close_verified\":true")!=std::string::npos,"all worker descriptors closed");
    if(scenario==4)check(read_text(output/"supervisor.json").find("\"collection_complete\":true")!=std::string::npos,"content diff keeps complete collection");
  }
}
}
int main(){try{relay_happy();partial_first_init();admission_and_supervisor();std::cout<<"PASS "<<checks<<" duplex Linux PTY/admission/supervision checks\n";return 0;}catch(const std::exception& error){std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}}
