// Linux production-port and CLI seam tests. Only newly created PTYs/files.
#define main observer_cli_entry
#include "main.cpp"
#undef main
#include "mech_bringup/pass_through_init.hpp"
#include <deque>
#include <exception>

namespace {std::deque<int> actions;}
extern "C" ssize_t __real_write(int,const void*,size_t);
extern "C" ssize_t __wrap_write(int fd,const void* p,size_t n){
  if(fd<=2||actions.empty())return __real_write(fd,p,n);
  auto action=actions.front();actions.pop_front();if(action<0){errno=-action;return -1;}
  return action==0?0:__real_write(fd,p,std::min(n,static_cast<std::size_t>(action)));
}
namespace {
using namespace observer;
using namespace mech::mech_control_core;
using mech::mech_bringup::PosixCdcSerialPort;
unsigned checks=0;
void verify(bool yes,const char* what){++checks;if(!yes)throw std::runtime_error(what);}
template<class F>void rejects(F action){bool caught=false;try{action();}catch(const std::exception&){caught=true;}verify(caught,"expected rejection");}
class Pty {
 public:
  int master{-1};std::string slave;
  Pty(){master=::posix_openpt(O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);verify(master>=0,"PTY open");verify(::grantpt(master)==0&&::unlockpt(master)==0,"PTY unlock");char path[256]{};verify(::ptsname_r(master,path,sizeof(path))==0,"PTY name");slave=path;}
  ~Pty(){if(master>=0)::close(master);}
  Bytes drain(std::size_t wanted){Bytes out;std::array<std::uint8_t,1024> b{};auto end=Clock::now()+std::chrono::seconds(2);while(out.size()<wanted&&Clock::now()<end){auto n=::read(master,b.data(),std::min(b.size(),wanted-out.size()));if(n>0)out.insert(out.end(),b.begin(),b.begin()+n);else if(n<0&&errno!=EAGAIN&&errno!=EINTR)break;else std::this_thread::yield();}return out;}
};
Bytes initialization(){const auto& init=mech::mech_bringup::kPassThroughInitFrame;return Bytes(init.begin(),init.end());}
void observe_only_initialization(){
  Pty p;PosixCdcSerialPort port(p.slave);verify(port.open(),"production PTY open");Config c;c.seconds=1;Capture capture(2*1024*1024);
  Bytes init;std::exception_ptr error;
  const Bytes rx{0xf7,0x12,0x0e,0,0x0b,0x93,0xf6,0x68,0x29,0,0,0x0c,8,3,0x44,0,0,0xff,0xfa,0x2c,0};
  std::thread peer([&]{try{init=p.drain(13);for(auto byte:rx){if(__real_write(p.master,&byte,1)!=1)throw std::runtime_error("owned PTY write");std::this_thread::sleep_for(std::chrono::milliseconds(1));}}catch(...){error=std::current_exception();}});
  auto result=run(port,c,capture,[]{return false;});peer.join();if(error)std::rethrow_exception(error);
  verify(result.complete()&&result.raw_complete(),"observer complete");verify(init==initialization(),"exact initialization");verify(result.tx_frames==0&&result.write_calls==1,"observe emitted no CAN writes");verify(result.rx_frames==1,"fragmented production read");verify(!port.is_open(),"observe port closed");
  std::array<std::uint8_t,32> extra{};verify(::read(p.master,extra.data(),extra.size())<0&&(errno==EAGAIN||errno==EIO),"no writes after initialization");
  PosixCdcSerialPort reopened(p.slave);verify(reopened.open(),"observer released lock");
}
void partial_init_closes(){
  for(const int transient:{-EAGAIN,0}){
    Pty p;PosixCdcSerialPort port(p.slave);verify(port.open(),"partial PTY open");Config c;c.seconds=1;Capture capture(8192);unsigned ready=0;
    actions={5,transient};auto result=run(port,c,capture,[]{return false;},[&]{++ready;});actions.clear();
    const auto expected=initialization();
    verify(!result.complete()&&result.error=="write_incomplete_no_retry","partial init faults");verify(!port.is_open()&&ready==0,"partial init closes without readiness");verify(p.drain(5)==Bytes(expected.begin(),expected.begin()+5),"only init prefix emitted");
  }
}
struct Temp {
  fs::path path;
  Temp(){char name[]="/tmp/observer-offline-test-XXXXXX";auto made=::mkdtemp(name);verify(made!=nullptr,"temporary tree");path=made;}
  ~Temp(){std::error_code error;fs::remove_all(path,error);}
};
std::string read_text(const fs::path& p){std::ifstream in(p);verify(in.is_open(),"metadata read");std::ostringstream out;out<<in.rdbuf();return out.str();}
void admission_seams(){
  Temp t;Pty p;Identity identity;struct stat st{};verify(::stat(p.slave.c_str(),&st)==0,"owned PTY stat");identity.device=st.st_rdev;identity.usb="/fake/usb/1-3.2";
  fs::create_directories(t.path/"task"/"fd");inspect_fds(t.path/"task",identity);inspect_fds(t.path/"exited",identity);
  fs::create_directory(t.path/"task-no-fd");rejects([&]{inspect_fds(t.path/"task-no-fd",identity);});
  fs::create_symlink(p.slave,t.path/"task"/"fd"/"8");rejects([&]{inspect_fds(t.path/"task",identity);});
  // Fake sysfs maps another descriptor to an interface of the same physical
  // board; admission must reject it even when its device number differs.
  auto sibling=identity;sibling.device=makedev(999,1);sibling.usb=(t.path/"usb"/"1-3.2").string();
  fs::create_directories(fs::path(sibling.usb)/"1-3.2:1.1");
  auto sysroot=t.path/"sys-char";auto sysdev=sysroot/(std::to_string(major(st.st_rdev))+":"+std::to_string(minor(st.st_rdev)));
  fs::create_directories(sysdev);fs::create_symlink(fs::path(sibling.usb)/"1-3.2:1.1",sysdev/"device");
  rejects([&]{inspect_fds(t.path/"task",sibling,sysroot);});
  sibling.usb=(t.path/"usb"/"1-3.3").string();inspect_fds(t.path/"task",sibling,sysroot);
  rejects([&]{identify(p.slave);});auto other=identity;verify(same_identity(identity,other),"identity equality");other.usb="/fake/usb/1-3.3";verify(!same_identity(identity,other),"physical identity change");
  write_new(t.path/"new-evidence","first");rejects([&]{write_new(t.path/"new-evidence","replacement");});verify(read_text(t.path/"new-evidence")=="first","evidence never overwritten");
  fs::create_symlink(t.path/"new-evidence",t.path/"symlink-evidence");rejects([&]{write_new(t.path/"symlink-evidence","bad");});
}
void reaped(pid_t child){int status=0;errno=0;verify(::waitpid(child,&status,WNOHANG)==-1&&errno==ECHILD,"worker reaped");}
void supervision(){
  Temp t;
  for(unsigned scenario=0;scenario<5;++scenario){
    auto output=t.path/std::to_string(scenario);fs::create_directory(output);stopped=0;auto child=::fork();verify(child>=0,"fake worker fork");
    if(child==0){
      if(scenario==4){::signal(SIGTERM,SIG_IGN);for(;;)::pause();}
      try{write_new(output/"plan.json","{}\n");write_new(output/"summary.json","{}\n");write_new(output/"capture.bin","");if(scenario!=1)write_new(output/"ready.json","{\"ready\":true}\n");}catch(...){::_exit(3);}
      ::_exit(scenario==2?1:(scenario==3?3:0));
    }
    auto start=Clock::now();auto result=supervise(child,output,"{}",std::chrono::milliseconds(scenario==4?150:1000));
    verify(Clock::now()-start<std::chrono::seconds(3),"bounded supervision");verify(result==(scenario==0?0:1),"supervisor completion category");reaped(child);
    auto metadata=read_text(output/"supervisor.json");verify(metadata.find("\"ports_close_verified\":true")!=std::string::npos,"process descriptors closed");
    verify(metadata.find(std::string("\"export_valid\":")+((scenario==0||scenario==2)?"true":"false"))!=std::string::npos,"ready and export validity");
  }
  auto child=::fork();verify(child>=0,"exception guard fork");if(child==0)for(;;)::pause();
  rejects([&]{WorkerGuard guard{child};throw std::runtime_error("injected parent fault");});reaped(child);
}
}
int main(){try{observe_only_initialization();partial_init_closes();admission_seams();supervision();std::cout<<"PASS "<<checks<<" observer Linux PTY/admission/supervision checks\n";return 0;}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
