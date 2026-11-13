#pragma once

#include <unistd.h>
#include <array>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <type_traits>
#include <utility>
#include <vector>

namespace generation_metadata {

inline void jsonString(std::ostream& out, std::string_view value) {
  out << '"';
  const char* hex = "0123456789abcdef";
  for (unsigned char c : value) {
    if (c == '"' || c == '\\')
      out << '\\' << c;
    else if (c < 32)
      out << "\\u00" << hex[c >> 4] << hex[c & 15];
    else
      out << c;
  }
  out << '"';
}

class JsonObject {
 public:
  explicit JsonObject(std::ostream& out, int depth = 0) : out_(out), depth_(depth) { out_ << '{'; }
  ~JsonObject() { out_ << '\n' << std::string(depth_, ' ') << '}'; }
  void key(const std::string& name) {
    if (!first_) out_ << ',';
    first_ = false;
    out_ << '\n' << std::string(depth_ + 2, ' ');
    jsonString(out_, name);
    out_ << ": ";
  }
  template <typename T>
  void field(const std::string& name, const T& value) {
    key(name);
    if constexpr (std::is_same_v<T, std::nullptr_t>) out_ << "null";
    else if constexpr (std::is_convertible_v<T, std::string_view>) jsonString(out_, value);
    else if constexpr (std::is_same_v<T, bool>) out_ << (value ? "true" : "false");
    else out_ << std::setprecision(17) << value;
  }

 private:
  std::ostream& out_;
  int depth_;
  bool first_ = true;
};

inline std::string shellQuote(const std::string& value) {
  std::string quoted = "'";
  for (char c : value) quoted += c == '\'' ? "'\\''" : std::string(1, c);
  return quoted + "'";
}

inline std::string commandOutput(const std::string& command) {
  FILE* pipe = ::popen((command + " 2>/dev/null").c_str(), "r");
  if (!pipe) return {};
  std::string output;
  std::array<char, 4096> buffer;
  while (std::fgets(buffer.data(), buffer.size(), pipe)) output += buffer.data();
  if (::pclose(pipe) != 0) return {};
  while (!output.empty() && (output.back() == '\n' || output.back() == '\r')) output.pop_back();
  return output;
}

inline std::string fingerprint(const std::filesystem::path& path) {
  const auto value = commandOutput("sha256sum -- " + shellQuote(path.string()));
  const size_t offset = !value.empty() && value[0] == '\\' ? 1 : 0;
  return value.size() >= offset + 64 ? value.substr(offset, 64) : std::string();
}

inline std::string readText(const std::filesystem::path& path) {
  std::ifstream stream(path, std::ios::binary);
  if (!stream) throw std::runtime_error("Cannot read generation input: " + path.string());
  std::ostringstream text;
  text << stream.rdbuf();
  if (stream.bad()) throw std::runtime_error("Cannot read generation input: " + path.string());
  return text.str();
}

inline std::string utcTime() {
  const auto now = std::time(nullptr);
  std::ostringstream text;
  text << std::put_time(std::gmtime(&now), "%Y-%m-%dT%H:%M:%SZ");
  return text.str();
}

class GenerationManifest {
 public:
  using OptionsWriter = std::function<void(JsonObject&)>;
  GenerationManifest(const std::filesystem::path& outputDirectory,
                     const std::filesystem::path& sourceDirectory,
                     const std::vector<std::pair<std::string, std::string>>& inputFiles,
                     int argc,
                     char** argv,
                     OptionsWriter optionsWriter)
      : optionsWriter_(std::move(optionsWriter)), started_(utcTime()), cwd_(std::filesystem::current_path().string()) {
    std::filesystem::create_directories(outputDirectory);
    const auto stamp = std::chrono::system_clock::now().time_since_epoch().count();
    path_ =
        std::filesystem::absolute(outputDirectory) / ("generation_" + std::to_string(stamp) + "_" + std::to_string(::getpid()) + ".json");
    for (int i = 0; i < argc; ++i) arguments_.emplace_back(argv[i]);
    for (const auto& [name, inputPath] : inputFiles) {
      const auto path = std::filesystem::canonical(inputPath);
      inputs_.push_back({name, path.string(), readText(path), fingerprint(path)});
    }
    const auto git = "git -c " + shellQuote("safe.directory=" + sourceDirectory.string()) + " -C " + shellQuote(sourceDirectory.string());
    commit_ = commandOutput(git + " rev-parse HEAD");
    dirty_ = !commandOutput(git + " status --porcelain").empty();
    executable_ = std::filesystem::read_symlink("/proc/self/exe").string();
    executableHash_ = fingerprint("/proc/" + std::to_string(::getpid()) + "/exe");
    std::set<std::string> libraries;
    std::ifstream maps("/proc/self/maps");
    for (std::string line; std::getline(maps, line);) {
      const auto slash = line.find('/');
      if (slash == std::string::npos) continue;
      const auto path = line.substr(slash);
      const auto name = std::filesystem::path(path).filename().string();
      if (name.rfind("libblasfeo.so", 0) == 0 || name.rfind("libhpipm.so", 0) == 0) libraries.insert(path);
    }
    for (const auto& path : libraries) libraries_.push_back({path, fingerprint(path)});
    checkInputs();
    write("running");
  }

  const std::filesystem::path& path() const { return path_; }
  void checkInputs() const {
    for (const auto& input : inputs_) {
      if (readText(input.path) != input.contents) throw std::runtime_error("Generation input changed: " + input.path);
    }
  }
  void record(size_t index, const std::filesystem::path& file, uint32_t seed, size_t attempts) {
    motions_.push_back({index, std::filesystem::relative(std::filesystem::absolute(file), path_.parent_path()).string(), seed, attempts});
    write("running");
  }
  void complete() { write("completed"); }
  void fail(const std::string& error) { write("failed", error); }

 private:
  struct Input {
    std::string name, path, contents, hash;
  };
  struct Motion {
    size_t index;
    std::string file;
    uint32_t seed;
    size_t attempts;
  };
  void write(const std::string& status, const std::string& error = {}) const {
    const auto temporary = path_.string() + ".tmp";
    std::ofstream out(temporary, std::ios::binary | std::ios::trunc);
    if (!out) throw std::runtime_error("Cannot write generation manifest: " + path_.string());
    {
      JsonObject root(out);
      root.field("schema_version", 1);
      root.field("status", status);
      root.field("started_at_utc", started_);
      root.field("updated_at_utc", utcTime());
      root.field("completed_motions", motions_.size());
      if (!error.empty()) root.field("error", error);
      root.field("working_directory", cwd_);
      root.key("argv");
      out << '[';
      for (size_t i = 0; i < arguments_.size(); ++i) {
        if (i) out << ',';
        jsonString(out, arguments_[i]);
      }
      out << ']';
      root.key("options");
      {
        JsonObject options(out, 2);
        optionsWriter_(options);
      }
      root.key("source_tree");
      {
        JsonObject source(out, 2);
        if (commit_.empty()) {
          source.field("git_commit", nullptr);
          source.field("dirty", nullptr);
        } else {
          source.field("git_commit", commit_);
          source.field("dirty", dirty_);
        }
        source.field("generator_executable", executable_);
        if (executableHash_.empty())
          source.field("generator_sha256", nullptr);
        else
          source.field("generator_sha256", executableHash_);
      }
      root.key("inputs");
      {
        JsonObject inputs(out, 2);
        for (const auto& input : inputs_) {
          inputs.key(input.name);
          JsonObject entry(out, 4);
          entry.field("path", input.path);
          if (input.hash.empty())
            entry.field("sha256", nullptr);
          else
            entry.field("sha256", input.hash);
          entry.field("contents", input.contents);
        }
      }
      root.key("solver");
      {
        JsonObject solver(out, 2);
        solver.field("method", "SQP");
        solver.field("qp_solver", "HPIPM");
        solver.key("libraries");
        out << '[';
        for (size_t i = 0; i < libraries_.size(); ++i) {
          if (i) out << ',';
          JsonObject entry(out, 4);
          entry.field("path", libraries_[i].first);
          if (libraries_[i].second.empty())
            entry.field("sha256", nullptr);
          else
            entry.field("sha256", libraries_[i].second);
        }
        out << ']';
      }
      root.key("conventions");
      {
        JsonObject conventions(out, 2);
        conventions.field("time_unit", "s");
        conventions.field("joint_angle_unit", "rad");
        conventions.field("quaternion_order", "wxyz");
        conventions.field("row_alignment", "command and robot state at the same time[k]");
        conventions.field("reproducibility", "Wall-clock command filtering and numerical execution can affect trajectories");
      }
      root.key("motions");
      out << '[';
      for (size_t i = 0; i < motions_.size(); ++i) {
        if (i) out << ',';
        const auto& motion = motions_[i];
        JsonObject entry(out, 4);
        entry.field("index", motion.index);
        entry.field("file", motion.file);
        entry.field("seed", motion.seed);
        entry.field("attempts", motion.attempts);
      }
      out << ']';
    }
    out << '\n';
    out.close();
    if (!out) throw std::runtime_error("Failed writing generation manifest: " + path_.string());
    std::filesystem::rename(temporary, path_);
  }
  OptionsWriter optionsWriter_;
  std::filesystem::path path_;
  std::string started_, cwd_, commit_, executable_, executableHash_;
  bool dirty_ = false;
  std::vector<std::string> arguments_;
  std::vector<Input> inputs_;
  std::vector<std::pair<std::string, std::string>> libraries_;
  std::vector<Motion> motions_;
};
}  // namespace generation_metadata
