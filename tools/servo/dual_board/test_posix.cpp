// Linux-only offline production-port tests. Every endpoint is a new PTY;
// arbitrary PTYs remain forbidden to the operator CLI.
#define main diagnostic_cli_entry
#include "main.cpp"
#undef main
#include "mech_bringup/pass_through_init.hpp"
#include <algorithm>
#include <atomic>
#include <deque>
#include <exception>
#include <mutex>
#include <vector>
#include <sys/types.h>

namespace {
std::deque<int> write_actions;
}
extern "C" ssize_t __real_write(int,const void*,size_t);
extern "C" ssize_t __wrap_write(int fd,const void* data,size_t size) {
  if(fd<=2 || write_actions.empty())return __real_write(fd,data,size);
  const auto action=write_actions.front();write_actions.pop_front();
  if(action<0){errno=-action;return -1;}
  return action==0?0:__real_write(fd,data,std::min(size,static_cast<std::size_t>(action)));
}
namespace {
using namespace dual_board;
using mech::mech_bringup::PosixCdcSerialPort;
unsigned assertions=0;
void verify(bool yes,const char* message) {++assertions;if(!yes)throw std::runtime_error(message);}
template<class F>void rejects(F action,const std::string& category) {
  bool threw=false;
  try{action();}catch(const std::exception& e){threw=true;verify(std::string(e.what()).find(category)!=std::string::npos,"wrong rejection");}
  verify(threw,"expected rejection");
}
class Pty {
 public:
  int master{-1};std::string slave;
  Pty() {
    master=::posix_openpt(O_RDWR|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);
    verify(master>=0,"posix_openpt");verify(::grantpt(master)==0,"grantpt");verify(::unlockpt(master)==0,"unlockpt");
    char name[256]{};verify(::ptsname_r(master,name,sizeof(name))==0,"ptsname_r");slave=name;
  }
  ~Pty(){if(master>=0)::close(master);}
  Pty(const Pty&)=delete;
  Pty& operator=(const Pty&)=delete;
  Bytes drain(std::size_t wanted) {
    Bytes output;std::array<std::uint8_t,1024> data{};
    const auto end=Clock::now()+std::chrono::seconds(2);
    while(output.size()<wanted && Clock::now()<end) {
      auto n=::read(master,data.data(),std::min(data.size(),wanted-output.size()));
      if(n>0)output.insert(output.end(),data.begin(),data.begin()+n);
      else if(n<0 && errno!=EAGAIN && errno!=EINTR)throw std::runtime_error("PTY read error");
      else std::this_thread::yield();
    }
    return output;
  }
  void send(const Bytes& bytes) {
    std::size_t offset=0;const auto end=Clock::now()+std::chrono::seconds(2);
    while(offset<bytes.size() && Clock::now()<end) {
      auto n=__real_write(master,bytes.data()+offset,bytes.size()-offset);
      if(n>0)offset+=n;
      else if(n<0 && errno!=EAGAIN && errno!=EINTR)throw std::runtime_error("PTY write error");
      else std::this_thread::yield();
    }
    verify(offset==bytes.size(),"PTY write timeout");
  }
};
const Bytes golden{0xf7,0x12,0x0e,0,0x0b,0x91,0x60,0x68,6,0,0,0x0c,8,0,0x0c,0xc1,0xa0,0,0x0a,0,0x0a};
void production_partial_writes() {
  Pty p;PosixCdcSerialPort port(p.slave);verify(port.open(),"open PTY");
  write_actions={-EINTR,5,-EINTR,3,13};
  verify(port.write_all(golden.data(),golden.size())==TransportResult::Ok,"successful partial offsets");
  verify(p.drain(golden.size())==golden,"golden bytes after partial writes");
  for(const int result:{-EAGAIN,0}) {
    write_actions={result};
    verify(port.write_all(golden.data(),golden.size())==TransportResult::WouldBlock,"no-progress should block");
    write_actions={5,result};
    verify(port.write_all(golden.data(),golden.size())==TransportResult::Fault,"partial must fault");
    verify(p.drain(5)==Bytes(golden.begin(),golden.begin()+5),"partial prefix exact");
  }
  write_actions.clear();port.close();verify(!port.is_open(),"port close");
  std::size_t n=99;std::array<std::uint8_t,16> input{};
  verify(port.read_some(input.data(),input.size(),n)==TransportResult::Disconnected,"closed read");
  verify(port.write_all(golden.data(),golden.size())==TransportResult::Disconnected,"closed write");
}
void locking_and_fragmented_reads() {
  Pty p;PosixCdcSerialPort first(p.slave),second(p.slave);
  verify(first.open(),"first open");verify(!second.open(),"second endpoint lock must fail");
  const Bytes rx{0xf7,0x12,0x0e,0,0x0b,0x93,0xf6,0x68,0x29,0,0,0x0c,8,3,0x44,0,0,0xff,0xfa,0x2c,0};
  Parser parser;unsigned received=0;
  for(auto byte:rx) {
    p.send(Bytes{byte});const auto deadline=Clock::now()+std::chrono::seconds(2);bool got=false;
    while(Clock::now()<deadline && !got) {
      std::uint8_t input=0;std::size_t size=0;
      const auto outcome=first.read_some(&input,1,size);
      if(outcome==TransportResult::Ok && size==1) {
        parser.feed(&input,1,[&](const RawCanFrame& f){verify(f.id.value==0x2968,"fragmented ID");++received;});got=true;
      } else verify(outcome==TransportResult::WouldBlock || (outcome==TransportResult::Ok && size==0),"unexpected read outcome");
      std::this_thread::yield();
    }
    verify(got,"fragment delivery timeout");
  }
  parser.finish();verify(received==1,"single fragmented record");
  first.close();verify(second.open(),"lock released by close");second.close();
}
void run_actual_ports() {
  for(const std::string packing:{"separate","joined","batch"}) {
    Pty txpty,rxpty;PosixCdcSerialPort tx(txpty.slave),rx(rxpty.slave);
    verify(tx.open() && rx.open(),"open production pair");
    Config c;c.hz=1;c.seconds=1;c.lanes=2;c.drain_ms=200;c.nonce=123;c.packing=packing;
    Capture capture(1024*1024);std::atomic<bool> exit{false};std::exception_ptr emulator_error;
    std::thread emulator([&]{
      try {
        // Test-only link forwards sender envelopes into the receiver master.
        // Init commands are consumed on both PTYs without synthesizing CAN.
        Bytes pending;std::array<std::uint8_t,1024> bytes{};
        while(!exit.load()) {
          auto n=::read(txpty.master,bytes.data(),bytes.size());
          if(n>0)pending.insert(pending.end(),bytes.begin(),bytes.begin()+n);
          while(pending.size()>=7) {
            const auto length=static_cast<unsigned>(pending[2])+(static_cast<unsigned>(pending[3])<<8);
            if(pending.size()<length+7)break;
            Bytes packet(pending.begin(),pending.begin()+length+7);
            pending.erase(pending.begin(),pending.begin()+length+7);
            if(length!=6)rxpty.send(packet);
          }
          auto init_count=::read(rxpty.master,bytes.data(),bytes.size());
          if(init_count<0 && errno!=EAGAIN && errno!=EINTR)throw std::runtime_error("receiver master error");
          std::this_thread::sleep_for(std::chrono::microseconds(100));
        }
      }catch(...){emulator_error=std::current_exception();}
    });
    auto result=run(tx,rx,c,capture,[]{return false;});exit=true;emulator.join();
    if(emulator_error)std::rethrow_exception(emulator_error);
    verify(result.complete(),("run production pair: "+result.error).c_str());
    verify(!tx.is_open() && !rx.is_open(),"run closes both production ports");
    PosixCdcSerialPort reopen_tx(txpty.slave),reopen_rx(rxpty.slave);
    verify(reopen_tx.open() && reopen_rx.open(),"run releases both endpoint locks");
  }
  // The first CAN write has a five-byte prefix then EAGAIN. It must fault
  // and never emit the second ID or retry the prefix.
  Pty a,b;PosixCdcSerialPort tx(a.slave),rx(b.slave);
  verify(tx.open() && rx.open(),"open fault pair");Config c;c.hz=1;c.seconds=1;c.lanes=2;c.drain_ms=200;
  Capture capture(8192);write_actions={13,13,5,-EAGAIN};
  auto result=run(tx,rx,c,capture,[]{return false;});write_actions.clear();
  verify(result.error=="write_incomplete_no_retry","run production partial error");
  const auto& init=mech::mech_bringup::kPassThroughInitFrame;
  auto want=Bytes(init.begin(),init.end());const auto first=packets(c,0).front();
  want.insert(want.end(),first.begin(),first.begin()+5);
  verify(a.drain(want.size())==want,"production partial stream only");
  verify(b.drain(init.size())==Bytes(init.begin(),init.end()),"receiver init only");
  verify(!tx.is_open() && !rx.is_open(),"fault closes production pair");
}
void cli_helpers_with_fake_tree() {
  verify(decimal("4294967295")==4294967295U,"maximum nonce");
  rejects([]{decimal("4294967296");},"overflow");rejects([]{decimal("-1");},"digits");
  verify(numeric_name(fs::path("fake/123")),"numeric task name");
  verify(!numeric_name(fs::path("fake/123a")),"nonnumeric task name");
  verify(quote("a\n\"\\") == "\"a\\u000a\\\"\\\\\"","JSON escaping");
  char pattern[]="/tmp/dual-board-preflight-XXXXXX";
  const auto path=::mkdtemp(pattern);verify(path!=nullptr,"fake tree temp");const fs::path root(path);
  struct Remove {fs::path path;~Remove(){std::error_code error;fs::remove_all(path,error);}} remove{root};
  fs::create_directories(root/"task"/"fd");
  {std::ofstream file(root/"attribute");file<<"caf1\n";}
  verify(attribute(root/"attribute")=="caf1","fake sysfs attribute read");
  rejects([&]{attribute(root/"missing");},"cannot read");
  Identity a,b;a.device=makedev(999,1);b.device=makedev(999,2);
  inspect_fds(root/"task",a,b); // Empty fake task and exited task are permitted.
  inspect_fds(root/"exited",a,b);
  fs::create_directory(root/"live-no-fd");
  rejects([&]{inspect_fds(root/"live-no-fd",a,b);},"cannot inspect");
  Pty p;struct stat st{};verify(::stat(p.slave.c_str(),&st)==0,"stat only owned PTY");a.device=st.st_rdev;
  fs::create_symlink(p.slave,root/"task"/"fd"/"4");
  rejects([&]{inspect_fds(root/"task",a,b);},"already occupied");
  rejects([&]{identify(p.slave);},"direct /dev/serial/by-path");
}
std::string read_text(const fs::path& path) {
  std::ifstream file(path);verify(file.is_open(),"read supervisor evidence");
  std::ostringstream result;result<<file.rdbuf();return result.str();
}
void verify_reaped(pid_t child) {
  int status=0;errno=0;
  verify(::waitpid(child,&status,WNOHANG)==-1 && errno==ECHILD,"supervisor leaves no waitable child");
}
void supervisor_resource_cleanup() {
  char pattern[]="/tmp/dual-board-supervisor-XXXXXX";
  const auto path=::mkdtemp(pattern);verify(path!=nullptr,"supervisor temp");const fs::path root(path);
  struct Remove {fs::path path;~Remove(){std::error_code error;fs::remove_all(path,error);}} remove{root};
  for(unsigned scenario=0;scenario<5;++scenario) {
    const auto output=root/std::to_string(scenario);fs::create_directory(output);stopped=0;
    const auto child=::fork();verify(child>=0,"fake worker fork");
    if(child==0) {
      if(scenario==4){::signal(SIGTERM,SIG_IGN);for(;;)::pause();}
      if(scenario!=1) {
        try {write_new(output/"plan.json","{}\n");write_new(output/"summary.json","{}\n");write_new(output/"capture.bin","");}
        catch(...){::_exit(3);}
      }
      ::_exit(scenario==2?1:(scenario==3?3:0));
    }
    const auto start=Clock::now();
    const auto result=supervise(child,output,"{}",std::chrono::milliseconds(scenario==4?150:1000));
    verify(Clock::now()-start<std::chrono::seconds(3),"supervisor deadline bound");
    verify(result==(scenario==0?0:1),"supervisor exit category");verify_reaped(child);
    const auto summary=read_text(output/"supervisor.json");
    verify(summary.find("\"worker_reaped\":true")!=std::string::npos,"worker reaped metadata");
    verify(summary.find("\"ports_close_verified\":true")!=std::string::npos,"closed child descriptor metadata");
    verify(summary.find(std::string("\"complete\":")+(scenario==0?"true":"false"))!=std::string::npos,"completion metadata");
    verify(summary.find(std::string("\"export_valid\":")+((scenario==0||scenario==2)?"true":"false"))!=std::string::npos,"export metadata");
    verify(summary.find(std::string("\"forced_timeout\":")+(scenario==4?"true":"false"))!=std::string::npos,"timeout metadata");
    if(scenario==1 || scenario==4)
      verify(read_text(output/"summary.json").find("\"capture_recoverable\":false")!=std::string::npos,"fallback cannot claim capture");
  }
  {
    const auto output=root/"interrupted";fs::create_directory(output);
    const auto child=::fork();verify(child>=0,"signal worker fork");
    if(child==0){::signal(SIGTERM,SIG_DFL);for(;;)::pause();}
    stopped=1;verify(supervise(child,output,"{}",std::chrono::milliseconds(1000))==1,"interrupt terminates worker");
    stopped=0;verify_reaped(child);
    verify(read_text(output/"supervisor.json").find("\"forced_timeout\":false")!=std::string::npos,"signal exit needs no forced kill");
  }
  {
    const auto child=::fork();verify(child>=0,"exception guard fork");
    if(child==0){for(;;)::pause();}
    rejects([&]{WorkerGuard guard{child};throw std::runtime_error("injected parent exception");},"injected parent exception");
    verify_reaped(child);
  }
  {
    const auto child=::fork();verify(child>=0,"export error worker fork");
    if(child==0)::_exit(0);
    rejects([&]{supervise(child,root/"missing-parent"/"missing-output","{}",std::chrono::milliseconds(1000));},"cannot create evidence");
    verify_reaped(child);
  }
}
}
int main() {
  try {
    production_partial_writes();locking_and_fragmented_reads();run_actual_ports();cli_helpers_with_fake_tree();supervisor_resource_cleanup();
    std::cout<<"PASS: "<<assertions<<" Linux PTY/preflight checks (no hardware)\n";return 0;
  }catch(const std::exception& e){std::cerr<<"FAIL: "<<e.what()<<'\n';return 1;}
}
