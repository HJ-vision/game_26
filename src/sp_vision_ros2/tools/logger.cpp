#include "logger.hpp"

#include <fmt/chrono.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <cctype>
#include <chrono>
#include <cstdlib>
#include <string>

namespace tools
{
std::shared_ptr<spdlog::logger> logger_ = nullptr;

void set_logger()
{
  auto file_name = fmt::format("logs/{:%Y-%m-%d_%H-%M-%S}.log", std::chrono::system_clock::now());
  auto file_sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(file_name, true);
  file_sink->set_level(spdlog::level::debug);  // 文件保留全部细节，排查时看这里

  // 终端默认只显示 info 及以上：逐帧 debug 级诊断（DETECT-DIAG/TRACKER-OBS/...）
  // 只写文件不刷屏。需要在终端看全量诊断时：
  //   RM_LOG_LEVEL=debug ros2 launch ...
  spdlog::level::level_enum console_level = spdlog::level::info;
  if (const char * env = std::getenv("RM_LOG_LEVEL")) {
    std::string s = env;
    for (auto & c : s) c = static_cast<char>(std::tolower(static_cast<unsigned char>(c)));
    if (s == "debug" || s == "trace") console_level = spdlog::level::debug;
    else if (s == "warn" || s == "warning") console_level = spdlog::level::warn;
    else if (s == "err" || s == "error") console_level = spdlog::level::err;
  }
  auto console_sink = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
  console_sink->set_level(console_level);

  logger_ = std::make_shared<spdlog::logger>("", spdlog::sinks_init_list{file_sink, console_sink});
  logger_->set_level(spdlog::level::debug);
  logger_->flush_on(spdlog::level::warn);  // 只在 warn 及以上立刻刷盘，info 交给缓冲
}

std::shared_ptr<spdlog::logger> logger()
{
  if (!logger_) set_logger();
  return logger_;
}

}  // namespace tools
