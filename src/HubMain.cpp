#include "BlackBoard.hpp"
#include "Common.hpp"
#include "DataDesc.hpp"
#include "Hub.hpp"
#include "Timer.hpp"
#include "Utility.hpp"
#include <cctype>
#include <condition_variable>
#include <fstream>
#include <mutex>
#include <string>
#include <vector>

#ifdef ARTINXHUB_LINUX
#include <cfenv>
#endif

#include "SuppressWarningBegin.hpp"

#include <caf/actor_registry.hpp>
#include <caf/actor_system.hpp>
#include <caf/actor_system_config.hpp>
#include <caf/event_based_actor.hpp>
#include <caf/exec_main.hpp>
#include <caf/logger.hpp>
#include <caf/scoped_actor.hpp>
#include <fmt/format.h>

#include "SuppressWarningEnd.hpp"

using namespace std::literals;

static std::string loadConfig(const char* path) {
    std::ifstream in{ path, std::ios::in | std::ios::binary };
    const auto size = in.seekg(0, std::ios::end).tellg();
    std::string res(static_cast<size_t>(size), '\0');
    in.seekg(0, std::ios::beg).read(res.data(), size);
    return res;
}

template <typename String>
static void demangle(String& typeName) {
#ifdef ARTINXHUB_WINDOWS
    // For MSVC
    if(const auto pos = typeName.find_last_of(' '); pos != String::npos)
        typeName = typeName.substr(pos + 1);
#else
    // For GCC/Clang
    while(std::isdigit(typeName.front())) {
        typeName = typeName.substr(1);
    }
#endif  // ARTINXHUB_WINDOWS
}

class NodeFactory final : Unmovable {
    std::unordered_map<std::string, std::function<caf::actor(caf::actor_system&, const HubConfig&)>> mClasses{};

public:
    void addNodeType(std::string name, std::function<caf::actor(caf::actor_system&, const HubConfig&)> spawnFunction) {
        demangle(name);
        if(!mClasses.emplace(std::move(name), std::move(spawnFunction)).second) {
            logError("Multiple definition of node type " + name);
        }
    }
    caf::actor buildNode(caf::actor_system& system, const std::string& name, const HubConfig& config) {
        logInfo("Build node " + name);
        const auto attr = config.to_dictionary().value();
        const auto typeAttr = attr.find("type"sv);
        if(typeAttr == attr.cend()) {
            raiseError("Node " + name + " is lack of 'type' field.");
        }
        const auto nodeTypeName = caf::to_string(typeAttr->second);
        const auto iter = mClasses.find(nodeTypeName);
        if(iter == mClasses.cend()) {
            raiseError("Undefined Node Type " + nodeTypeName);
        }

        try {
            auto actor = iter->second(system, config);
            system.registry().put(name, actor);
            return actor;
        } catch(const std::exception& e) {
            logError(e.what());
            throw;
        }
    }
    static NodeFactory& get() {
        static NodeFactory instance;
        return instance;
    }
};

namespace detail {
    void registerComponent(const char* name, std::function<caf::actor(caf::actor_system&, const HubConfig&)> spawnFunction) {
        NodeFactory::get().addNodeType(std::string{ name }, std::move(spawnFunction));
    }

    std::vector<std::string> parseSucceed(const HubConfig& config, const std::string& name) {
        std::string_view nameNormalized = name;
        demangle(nameNormalized);
        const auto attr = config.to_dictionary().value();
        const auto iter = attr.find(nameNormalized);
        if(iter == attr.cend()) {
            return {};
        }

        const auto succeed = iter->second.to_list().value();
        std::vector<std::string> res;
        res.reserve(succeed.size());
        for(const auto& id : succeed) {
            res.push_back(caf::to_string(id));
        }
        return res;
    }

    static std::unordered_map<std::string, GroupMask> maskLUT;

    std::vector<std::pair<caf::actor_addr, GroupMask>> parseSucceed(caf::actor_system& system,
                                                                    const std::vector<std::string>& succeed) {
        const auto& registry = system.registry();
        std::vector<std::pair<caf::actor_addr, GroupMask>> res;
        res.reserve(succeed.size());
        for(const auto& id : succeed) {
            if(const auto addr = registry.get<caf::actor_addr>(id))
                res.emplace_back(addr, maskLUT[id]);
            else {
                logError("Undefined actor " + id + " (call sendAll before start_atom?)");
            }
        }
        return res;
    }
}  // namespace detail

std::vector<std::pair<std::string, caf::actor>> buildPipeline(caf::actor_system& system, const HubConfig& config) {
    const auto nodes = config.to_dictionary().value();
    std::unordered_map<std::string, uint32_t> idMap;
    std::vector<std::tuple<uint32_t, std::string, std::vector<uint32_t>>> reference;
    reference.reserve(nodes.size());

    auto&& factory = NodeFactory::get();

    std::vector<std::pair<std::string, caf::actor>> actors;
    actors.reserve(nodes.size());

    for(auto&& [name, sub] : nodes) {
        if(name == "global")
            continue;
        const auto& dict = sub.to_dictionary();

        if(const auto iter1 = dict->find("group_mask"); iter1 != dict->cend()) {
            detail::maskLUT[name] = static_cast<uint32_t>(iter1->second.to_integer().value());
        } else if(const auto iter2 = dict->find("group_id"); iter2 != dict->cend()) {
            detail::maskLUT[name] = 1U << static_cast<uint32_t>(iter2->second.to_integer().value());
        } else {
            detail::maskLUT[name] = 1U;
        }

        actors.emplace_back(name, factory.buildNode(system, name, sub));
    }

    return actors;
}

caf::actor createDaemonActor(caf::actor_system& sys, const std::vector<std::pair<std::string, caf::actor>>& actors);

RunStatus globalStatus = RunStatus::running;
static std::mutex globalMutex;
static std::condition_variable globalCV;

void terminateSystem(caf::local_actor&, const bool success) {
    globalStatus = success ? RunStatus::normalExit : RunStatus::failureExit;
    globalCV.notify_one();
}

std::string globalConfigName;

void installFPEProbe() {
#ifdef ARTINXHUB_DEBUG
#ifdef ARTINXHUB_WINDOWS
    _control87(_EM_DENORMAL | _EM_INEXACT | _EM_UNDERFLOW, _MCW_EM);
#else
    feenableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
#endif
#endif
}

void uninstallFPEProbe() {
#ifdef ARTINXHUB_DEBUG
#ifdef ARTINXHUB_WINDOWS
    _control87(_MCW_EM, _MCW_EM);
#else
    fedisableexcept(FE_DIVBYZERO | FE_INVALID | FE_OVERFLOW);
#endif
#endif
}

int caf_main(caf::actor_system& system, const caf::actor_system_config& config) {
    logInfo("Initializing");
    Timer::instance().bindSystem(system);

    const auto [argc, argv] = config.c_args_remainder();
    auto args = "Command Arguments: "s;
    for(int idx = 1; idx < argc; ++idx) {
        args += argv[idx];
        args += ' ';
    }
    logInfo(args);

    if(argc != 2 || !fs::exists(argv[1])) {
        logError(fmt::format("argc:{}, argv[1]:{}", argc, argv[1]));
        logError("No config file path argument or config file path do not exsits!");
        return 0;
    }

    globalConfigName = fs::path{ argv[1] }.filename().string();
    if(const auto pos = globalConfigName.find('.'); pos != std::string::npos)
        globalConfigName = globalConfigName.substr(0, pos);

    const auto configData = loadConfig(argv[1]);
    const auto pipelineConfig = caf::config_value::parse(configData).value();
    GlobalSettings::get() = caf::get_as<GlobalSettings>(pipelineConfig.to_dictionary().value()["global"]).value();

    {
        const auto actors = buildPipeline(system, pipelineConfig);
        const auto daemon = createDaemonActor(system, actors);

        const caf::scoped_actor caller{ system };
        for(auto&& [name, actor] : actors) {
            caller->send(actor, start_atom_v);
        }
        logInfo("ArtinxHub Started");

        {
            std::unique_lock<std::mutex> lock{ globalMutex };
            globalCV.wait(lock, [] { return globalStatus != RunStatus::running; });
        }

        logInfo("ArtinxHub Finished");

        for(auto& [name, actor] : actors) {
            system.registry().erase(name);
            caller->send_exit(actor, caf::exit_reason::user_shutdown);
        }
        caller->send_exit(daemon, caf::exit_reason::user_shutdown);
    }

    Timer::instance().stop();
    system.await_all_actors_done();

    return globalStatus == RunStatus::normalExit ? EXIT_SUCCESS : EXIT_FAILURE;
}

std::mutex HubLogger::mutex;
std::unordered_map<std::string, TimePoint> HubLogger::logs;
std::unordered_map<std::string, std::string> HubLogger::watches;

CAF_MAIN(caf::id_block::ArtinxHub)
