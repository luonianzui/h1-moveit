#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <stdexcept>
#include <string>
#include <fcntl.h>
#include <sys/syscall.h>
#include <unistd.h>
#include <gperftools/profiler.h>
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <sdf/Element.hh>

class TaskCpuProfile final : public gz::sim::System,
    public gz::sim::ISystemConfigure, public gz::sim::ISystemPreUpdate
{
  using Clock = std::chrono::steady_clock;
  std::string output;
  std::string ready;
  std::string done;
  Clock::time_point started;
  Clock::time_point lastCheck{};
  pid_t targetThread{0};
  bool active{false};
  bool finished{false};
  unsigned long long updates{0};

  void Fail(const std::string &reason)
  {
    finished = true;
    std::fprintf(stderr, "TASK_CPU_PROFILE_FAILED: %s\n", reason.c_str());
    const int descriptor = open((output + ".failure").c_str(), O_WRONLY | O_CREAT | O_EXCL, 0444);
    if (descriptor >= 0)
    {
      write(descriptor, reason.data(), reason.size());
      close(descriptor);
    }
  }

  static int IncludeThread(void *argument)
  {
    return syscall(SYS_gettid) == *static_cast<const pid_t *>(argument);
  }

  void Stop(const std::string &reason)
  {
    if (!active)
      return;
    ProfilerState state{};
    ProfilerGetCurrentState(&state);
    ProfilerFlush();
    ProfilerStop();
    active = false;
    finished = true;
    const std::string text = "{\"status\":\"PROFILE_STOPPED\",\"pid\":" +
        std::to_string(getpid()) + ",\"tid\":" + std::to_string(targetThread) +
        ",\"sampled_thread_is_process_main\":" + (targetThread == getpid() ? "true" : "false") +
        ",\"wall_s\":" + std::to_string(std::chrono::duration<double>(Clock::now() - started).count()) +
        ",\"samples\":" + std::to_string(state.samples_gathered) +
        ",\"preupdate_calls\":" + std::to_string(updates) +
        ",\"reason\":\"" + reason + "\",\"no_simulation_commands\":true}\n";
    const int descriptor = open((output + ".complete").c_str(), O_WRONLY | O_CREAT | O_EXCL, 0444);
    if (descriptor >= 0)
    {
      const auto written = write(descriptor, text.data(), text.size());
      close(descriptor);
      if (written != static_cast<ssize_t>(text.size()))
        Fail("Incomplete unique profile metadata");
    }
    else
      Fail("Cannot exclusively create profile completion");
  }

public:
  ~TaskCpuProfile() override
  {
    try { Stop("server_destructor"); } catch (...) {}
  }

  void Configure(const gz::sim::Entity &, const std::shared_ptr<const sdf::Element> &element,
      gz::sim::EntityComponentManager &, gz::sim::EventManager &) override
  {
    output = element->Get<std::string>("profile_path");
    ready = element->Get<std::string>("ready_path");
    done = element->Get<std::string>("done_path");
    if (std::getenv("CPUPROFILE") || std::getenv("CPUPROFILE_REALTIME"))
      throw std::runtime_error("Global or wall-time profiler injection forbidden");
    for (const auto &path : {output, ready, done})
      if (path.empty() || !std::filesystem::path(path).is_absolute())
        throw std::runtime_error("Absolute owned task paths required");
    if (std::filesystem::exists(output) || std::filesystem::exists(output + ".complete") ||
        std::filesystem::exists(ready) || std::filesystem::exists(done))
      throw std::runtime_error("Native profile requires unused task identity");
    const int claim = open((output + ".claim").c_str(), O_WRONLY | O_CREAT | O_EXCL, 0444);
    if (claim < 0)
      throw std::runtime_error("Native profile identity already used");
    close(claim);
  }

  void PreUpdate(const gz::sim::UpdateInfo &, gz::sim::EntityComponentManager &) override
  {
    if (finished)
      return;
    const auto now = Clock::now();
    if (active)
      ++updates;
    if (now - lastCheck < std::chrono::milliseconds(100))
      return;
    lastCheck = now;
    if (active && (std::filesystem::exists(done) || now - started > std::chrono::seconds(900)))
    {
      Stop(std::filesystem::exists(done) ? "actual_task_result_written" : "bounded_wall_timeout");
      return;
    }
    if (!active && std::filesystem::exists(done))
    {
      Fail("Task ended before profile started");
      return;
    }
    if (!active && std::filesystem::exists(ready))
    {
      if (std::filesystem::exists(output) || std::filesystem::exists(done) ||
          ProfilingIsEnabledForAllThreads())
      {
        Fail("Profile output/task changed or another profiler active");
        return;
      }
      targetThread = static_cast<pid_t>(syscall(SYS_gettid));
      ProfilerOptions options{};
      options.filter_in_thread = IncludeThread;
      options.filter_in_thread_arg = &targetThread;
      if (!ProfilerStartWithOptions(output.c_str(), &options))
      {
        Fail("Cannot start owned task profile");
        return;
      }
      started = now;
      active = true;
    }
  }
};

GZ_ADD_PLUGIN(TaskCpuProfile, gz::sim::System,
              TaskCpuProfile::ISystemConfigure, TaskCpuProfile::ISystemPreUpdate)
