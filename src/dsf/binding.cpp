#include "./headers/AdjacencyMatrix.hpp"
#include "./headers/RoadNetwork.hpp"
#include "./headers/FirstOrderDynamics.hpp"
#include "./utility/Logger.hpp"

#include "./.docstrings.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>         // Changed to include all stl type casters
#include <pybind11/functional.h>  // For std::function support

PYBIND11_MODULE(dsf, m) {
  m.doc() = "Python bindings for the DSM library";

  // Create type aliases for better stub generation
  using WeightFunction = std::function<double(const dsf::RoadNetwork*, dsf::Id, dsf::Id)>;

  // Bind log_level_t enum
  pybind11::enum_<dsf::log_level_t>(m, "LogLevel")
      .value("DEBUG", dsf::log_level_t::DEBUG)
      .value("WARNING", dsf::log_level_t::WARNING)
      .value("INFO", dsf::log_level_t::INFO)
      .value("ERROR", dsf::log_level_t::ERROR)
      .export_values();

  // Bind TrafficLightOptimization enum
  pybind11::enum_<dsf::TrafficLightOptimization>(m, "TrafficLightOptimization")
      .value("SINGLE_TAIL", dsf::TrafficLightOptimization::SINGLE_TAIL)
      .value("DOUBLE_TAIL", dsf::TrafficLightOptimization::DOUBLE_TAIL)
      .export_values();

  // Bind Logger class
  pybind11::class_<dsf::Logger>(m, "Logger")
      .def_static("setVerbose", &dsf::Logger::setVerbose, pybind11::arg("verbose"))
      .def_static("setLogLevel", &dsf::Logger::setLogLevel, pybind11::arg("logLevel"))
      .def_static(
          "buildExceptionMessage",
          [](const std::string& message) {
            return dsf::Logger::buildExceptionMessage(message);
          },
          pybind11::arg("message"))
      .def_static(
          "debug",
          [](const std::string& message) { dsf::Logger::debug(message); },
          pybind11::arg("message"))
      .def_static(
          "info",
          [](const std::string& message) { dsf::Logger::info(message); },
          pybind11::arg("message"))
      .def_static(
          "warning",
          [](const std::string& message) { dsf::Logger::warning(message); },
          pybind11::arg("message"))
      .def_static(
          "error",
          [](const std::string& message) { dsf::Logger::error(message); },
          pybind11::arg("message"));
  pybind11::class_<dsf::Measurement<double>>(m, "Measurement")
      .def(pybind11::init<double, double>(),
           pybind11::arg("mean"),
           pybind11::arg("std"),
           dsf::g_docstrings.at("dsf::Measurement::Measurement").c_str())
      .def_readwrite("mean",
                     &dsf::Measurement<double>::mean,
                     dsf::g_docstrings.at("dsf::Measurement::mean").c_str())
      .def_readwrite("std",
                     &dsf::Measurement<double>::std,
                     dsf::g_docstrings.at("dsf::Measurement::std").c_str());

  pybind11::class_<dsf::AdjacencyMatrix>(m, "AdjacencyMatrix")
      .def(pybind11::init<>(),
           dsf::g_docstrings.at("dsf::AdjacencyMatrix::AdjacencyMatrix").c_str())
      .def(pybind11::init<std::string const&>(),
           pybind11::arg("fileName"),
           dsf::g_docstrings.at("dsf::AdjacencyMatrix::AdjacencyMatrix")
               .c_str())  // Added constructor
      .def("n",
           &dsf::AdjacencyMatrix::n,
           dsf::g_docstrings.at("dsf::AdjacencyMatrix::n").c_str())
      .def("size",
           &dsf::AdjacencyMatrix::size,
           dsf::g_docstrings.at("dsf::AdjacencyMatrix::size").c_str())
      .def("empty",
           &dsf::AdjacencyMatrix::empty,
           dsf::g_docstrings.at("dsf::AdjacencyMatrix::empty").c_str())  // Added empty
      .def("getRow",
           &dsf::AdjacencyMatrix::getRow,
           dsf::g_docstrings.at("dsf::AdjacencyMatrix::getRow").c_str())
      .def("getCol",
           &dsf::AdjacencyMatrix::getCol,
           dsf::g_docstrings.at("dsf::AdjacencyMatrix::getCol").c_str())  // Added getCol
      .def(
          "__call__",
          [](const dsf::AdjacencyMatrix& self, dsf::Id i, dsf::Id j) {
            return self(i, j);
          },
          dsf::g_docstrings.at("dsf::AdjacencyMatrix::operator()").c_str())
      .def("insert",
           &dsf::AdjacencyMatrix::insert,
           dsf::g_docstrings.at("dsf::AdjacencyMatrix::insert").c_str())  // Added insert
      .def("contains",
           &dsf::AdjacencyMatrix::contains,
           dsf::g_docstrings.at("dsf::AdjacencyMatrix::contains")
               .c_str())  // Added contains
      .def("elements",
           &dsf::AdjacencyMatrix::elements,
           dsf::g_docstrings.at("dsf::AdjacencyMatrix::elements")
               .c_str())  // Added elements
      .def("clear",
           &dsf::AdjacencyMatrix::clear,
           dsf::g_docstrings.at("dsf::AdjacencyMatrix::clear").c_str())
      .def("clearRow",
           &dsf::AdjacencyMatrix::clearRow,
           dsf::g_docstrings.at("dsf::AdjacencyMatrix::clearRow")
               .c_str())  // Added clearRow
      .def("clearCol",
           &dsf::AdjacencyMatrix::clearCol,
           dsf::g_docstrings.at("dsf::AdjacencyMatrix::clearCol")
               .c_str())  // Added clearCol
      .def("getInDegreeVector",
           &dsf::AdjacencyMatrix::getInDegreeVector,
           dsf::g_docstrings.at("dsf::AdjacencyMatrix::getInDegreeVector")
               .c_str())  // Added getInDegreeVector
      .def("getOutDegreeVector",
           &dsf::AdjacencyMatrix::getOutDegreeVector,
           dsf::g_docstrings.at("dsf::AdjacencyMatrix::getOutDegreeVector")
               .c_str())  // Added getOutDegreeVector
      .def("read",
           &dsf::AdjacencyMatrix::read,
           pybind11::arg("fileName"),
           dsf::g_docstrings.at("dsf::AdjacencyMatrix::read").c_str())  // Added read
      .def("save",
           &dsf::AdjacencyMatrix::save,
           pybind11::arg("fileName"),
           dsf::g_docstrings.at("dsf::AdjacencyMatrix::save").c_str());  // Added save

  pybind11::class_<dsf::RoadNetwork>(m, "RoadNetwork")
      .def(pybind11::init<>(),
           dsf::g_docstrings.at("dsf::RoadNetwork::RoadNetwork").c_str())
      .def(pybind11::init<const dsf::AdjacencyMatrix&>(),
           dsf::g_docstrings.at("dsf::RoadNetwork::RoadNetwork").c_str())
      .def("adjustNodeCapacities",
           &dsf::RoadNetwork::adjustNodeCapacities,
           dsf::g_docstrings.at("dsf::RoadNetwork::adjustNodeCapacities").c_str())
      .def("initTrafficLights",
           &dsf::RoadNetwork::initTrafficLights,
           pybind11::arg("minGreenTime") = 30,
           dsf::g_docstrings.at("dsf::RoadNetwork::initTrafficLights").c_str())
      .def("autoMapStreetLanes",
           &dsf::RoadNetwork::autoMapStreetLanes,
           dsf::g_docstrings.at("dsf::RoadNetwork::autoMapStreetLanes").c_str())
      .def("importMatrix",
           &dsf::RoadNetwork::importMatrix,
           pybind11::arg("fileName"),
           pybind11::arg("isAdj") = true,
           pybind11::arg("defaultSpeed") = 13.8888888889,
           dsf::g_docstrings.at("dsf::RoadNetwork::importMatrix").c_str())
      .def("importCoordinates",
           &dsf::RoadNetwork::importCoordinates,
           pybind11::arg("fileName"),
           dsf::g_docstrings.at("dsf::RoadNetwork::importCoordinates").c_str())
      .def("importOSMNodes",
           &dsf::RoadNetwork::importOSMNodes,
           pybind11::arg("fileName"),
           dsf::g_docstrings.at("dsf::RoadNetwork::importOSMNodes").c_str())
      .def("importOSMEdges",
           &dsf::RoadNetwork::importOSMEdges,
           pybind11::arg("fileName"),
           dsf::g_docstrings.at("dsf::RoadNetwork::importOSMEdges").c_str())
      .def("importTrafficLights",
           &dsf::RoadNetwork::importTrafficLights,
           pybind11::arg("fileName"),
           dsf::g_docstrings.at("dsf::RoadNetwork::importTrafficLights").c_str())
      .def("exportNodes",
           &dsf::RoadNetwork::exportNodes,
           pybind11::arg("fileName"),
           dsf::g_docstrings.at("dsf::RoadNetwork::exportNodes").c_str())
      .def("exportEdges",
           &dsf::RoadNetwork::exportEdges,
           pybind11::arg("fileName"),
           dsf::g_docstrings.at("dsf::RoadNetwork::exportEdges").c_str());

  pybind11::class_<dsf::Itinerary>(m, "Itinerary")
      .def(pybind11::init<dsf::Id, dsf::Id>(),
           pybind11::arg("id"),
           pybind11::arg("destination"),
           dsf::g_docstrings.at("dsf::Itinerary::Itinerary").c_str())
      .def("setPath",
           &dsf::Itinerary::setPath,
           pybind11::arg("path"),
           dsf::g_docstrings.at("dsf::Itinerary::setPath").c_str())
      .def("id", &dsf::Itinerary::id, dsf::g_docstrings.at("dsf::Itinerary::id").c_str())
      .def("destination",
           &dsf::Itinerary::destination,
           dsf::g_docstrings.at("dsf::Itinerary::destination").c_str());
  // .def("path", &dsf::Itinerary::path, pybind11::return_value_policy::reference_internal);

  pybind11::class_<dsf::FirstOrderDynamics>(m, "Dynamics")
      //     // Constructors are not directly exposed due to the template nature and complexity.
      //     // Users should use derived classes like FirstOrderDynamics.
      .def(pybind11::init<dsf::RoadNetwork&>(),
           pybind11::arg("graph"),
           dsf::g_docstrings.at("dsf::FirstOrderDynamics::FirstOrderDynamics").c_str())
      .def(pybind11::init<dsf::RoadNetwork&, bool>(),
           pybind11::arg("graph"),
           pybind11::arg("useCache"),
           dsf::g_docstrings.at("dsf::FirstOrderDynamics::FirstOrderDynamics").c_str())
      .def(pybind11::init<dsf::RoadNetwork&, bool, std::optional<unsigned int>>(),
           pybind11::arg("graph"),
           pybind11::arg("useCache"),
           pybind11::arg("seed"),
           dsf::g_docstrings.at("dsf::FirstOrderDynamics::FirstOrderDynamics").c_str())
      .def(pybind11::init<dsf::RoadNetwork&, bool, std::optional<unsigned int>, double>(),
           pybind11::arg("graph"),
           pybind11::arg("useCache"),
           pybind11::arg("seed"),
           pybind11::arg("alpha"),
           dsf::g_docstrings.at("dsf::FirstOrderDynamics::FirstOrderDynamics").c_str())
      .def(pybind11::init<dsf::RoadNetwork&, bool, std::optional<unsigned int>, double>(),
           pybind11::arg("graph"),
           pybind11::arg("useCache"),
           pybind11::arg("seed"),
           pybind11::arg("alpha"),
           dsf::g_docstrings.at("dsf::FirstOrderDynamics::FirstOrderDynamics").c_str())
      // Note: Constructors with std::function parameters are not exposed to avoid stub generation issues
      .def("setForcePriorities",
           &dsf::FirstOrderDynamics::setForcePriorities,
           pybind11::arg("forcePriorities"),
           dsf::g_docstrings.at("dsf::RoadDynamics::setForcePriorities").c_str())
      .def(
          "setDataUpdatePeriod",
          [](dsf::FirstOrderDynamics& self, int dataUpdatePeriod) {
            self.setDataUpdatePeriod(static_cast<dsf::Delay>(dataUpdatePeriod));
          },
          pybind11::arg("dataUpdatePeriod"),
          dsf::g_docstrings.at("dsf::RoadDynamics::setDataUpdatePeriod").c_str())
      .def("setMaxDistance",
           &dsf::FirstOrderDynamics::setMaxDistance,
           pybind11::arg("maxDistance"),
           dsf::g_docstrings.at("dsf::RoadDynamics::setMaxDistance").c_str())
      .def(
          "setMaxTravelTime",
          [](dsf::FirstOrderDynamics& self, uint64_t maxTravelTime) {
            self.setMaxTravelTime(static_cast<dsf::Time>(maxTravelTime));
          },
          pybind11::arg("maxTravelTime"),
          dsf::g_docstrings.at("dsf::RoadDynamics::setMaxTravelTime").c_str())
      .def("setErrorProbability",
           &dsf::FirstOrderDynamics::setErrorProbability,
           pybind11::arg("errorProbability"),
           dsf::g_docstrings.at("dsf::RoadDynamics::setErrorProbability").c_str())
      .def("setWeightFunction",
           &dsf::FirstOrderDynamics::setWeightFunction,
           pybind11::arg("strWeightFunction"))
      .def(
          "setDestinationNodes",
          [](dsf::FirstOrderDynamics& self,
             const std::vector<uint32_t>& destinationNodes,
             bool updatePaths) {
            self.setDestinationNodes(destinationNodes, updatePaths);
          },
          pybind11::arg("destinationNodes"),
          pybind11::arg("updatePaths") = true,
          dsf::g_docstrings.at("dsf::RoadDynamics::setDestinationNodes").c_str())
      .def("updatePaths",
           &dsf::FirstOrderDynamics::updatePaths,
           dsf::g_docstrings.at("dsf::RoadDynamics::updatePaths").c_str())
      .def("addAgentsUniformly",
           &dsf::FirstOrderDynamics::addAgentsUniformly,
           pybind11::arg("nAgents"),
           pybind11::arg("itineraryId") = std::nullopt,
           dsf::g_docstrings.at("dsf::RoadDynamics::addAgentsUniformly").c_str())
      .def(
          "addAgentsRandomly",
          [](dsf::FirstOrderDynamics& self,
             dsf::Size nAgents,
             const std::variant<std::monostate, size_t, double>& minNodeDistance) {
            self.addAgentsRandomly(nAgents, minNodeDistance);
          },
          pybind11::arg("nAgents"),
          pybind11::arg("minNodeDistance") = std::monostate{},
          dsf::g_docstrings.at("dsf::RoadDynamics::addAgentsRandomly").c_str())
      .def(
          "addAgentsRandomly",
          [](dsf::FirstOrderDynamics& self,
             dsf::Size nAgents,
             const std::unordered_map<dsf::Id, double>& src_weights,
             const std::unordered_map<dsf::Id, double>& dst_weights,
             const std::variant<std::monostate, size_t, double>& minNodeDistance) {
            self.addAgentsRandomly(nAgents, src_weights, dst_weights, minNodeDistance);
          },
          pybind11::arg("nAgents"),
          pybind11::arg("src_weights"),
          pybind11::arg("dst_weights"),
          pybind11::arg("minNodeDistance"),
          dsf::g_docstrings.at("dsf::RoadDynamics::addAgentsRandomly").c_str())
      .def(
          "addAgentsRandomly",
          [](dsf::FirstOrderDynamics& self,
             dsf::Size nAgents,
             const std::map<dsf::Id, double>& src_weights,
             const std::map<dsf::Id, double>& dst_weights,
             const std::variant<std::monostate, size_t, double>& minNodeDistance) {
            self.addAgentsRandomly(nAgents, src_weights, dst_weights, minNodeDistance);
          },
          pybind11::arg("nAgents"),
          pybind11::arg("src_weights"),
          pybind11::arg("dst_weights"),
          pybind11::arg("minNodeDistance"),
          dsf::g_docstrings.at("dsf::RoadDynamics::addAgentsRandomly").c_str())
      // .def("addAgent", static_cast<void (dsf::FirstOrderDynamics::*)(std::unique_ptr<dsf::Agent>)>(&dsf::FirstOrderDynamics::addAgent), pybind11::arg("agent"))
      // .def("addItinerary", static_cast<void (dsf::FirstOrderDynamics::*)(dsf::Id, dsf::Id)>(&dsf::FirstOrderDynamics::addItinerary), pybind11::arg("id"), pybind11::arg("destination"))
      // .def("addItinerary", static_cast<void (dsf::FirstOrderDynamics::*)(std::unique_ptr<dsf::Itinerary>)>(&dsf::FirstOrderDynamics::addItinerary), pybind11::arg("itinerary"))
      .def("evolve",
           &dsf::FirstOrderDynamics::evolve,
           pybind11::arg("reinsert_agents") = false,
           dsf::g_docstrings.at("dsf::RoadDynamics::evolve").c_str())
      .def("optimizeTrafficLights",
           &dsf::FirstOrderDynamics::optimizeTrafficLights,
           pybind11::arg("optimizationType") = dsf::TrafficLightOptimization::DOUBLE_TAIL,
           pybind11::arg("logFile") = "",
           pybind11::arg("threshold") = 0.,
           pybind11::arg("ratio") = 1.3,
           dsf::g_docstrings.at("dsf::RoadDynamics::optimizeTrafficLights").c_str())
      // .def("itineraries", &dsf::FirstOrderDynamics::itineraries, pybind11::return_value_policy::reference_internal)
      // .def("transitionMatrix", &dsf::FirstOrderDynamics::transitionMatrix, pybind11::return_value_policy::reference_internal)
      // .def("agents", &dsf::FirstOrderDynamics::agents, pybind11::return_value_policy::reference_internal)
      .def("nAgents",
           &dsf::FirstOrderDynamics::nAgents,
           dsf::g_docstrings.at("dsf::RoadDynamics::nAgents").c_str())
      .def("meanTravelTime",
           &dsf::FirstOrderDynamics::meanTravelTime,
           pybind11::arg("clearData") = false,
           dsf::g_docstrings.at("dsf::RoadDynamics::meanTravelTime").c_str())
      .def("meanTravelDistance",
           &dsf::FirstOrderDynamics::meanTravelDistance,
           pybind11::arg("clearData") = false,
           dsf::g_docstrings.at("dsf::RoadDynamics::meanTravelDistance").c_str())
      .def("meanTravelSpeed",
           &dsf::FirstOrderDynamics::meanTravelSpeed,
           pybind11::arg("clearData") = false,
           dsf::g_docstrings.at("dsf::RoadDynamics::meanTravelSpeed").c_str())
      //  .def("turnCounts",
      //       [](const dsf::FirstOrderDynamics& self) -> std::unordered_map<uint32_t, std::array<uint64_t, 4>> {
      //         const auto& counts = self.turnCounts();
      //         std::unordered_map<uint32_t, std::array<uint64_t, 4>> result;
      //         for (const auto& [id, arr] : counts) {
      //           result[id] = {arr[0], arr[1], arr[2], arr[3]};
      //         }
      //         return result;
      //       },
      //       "Get turn counts for all streets")
      //  .def("turnProbabilities",
      //       &dsf::FirstOrderDynamics::turnProbabilities,
      //       pybind11::arg("reset") = true)
      //  .def("turnMapping",
      //       &dsf::FirstOrderDynamics::turnMapping,
      //       pybind11::return_value_policy::reference_internal)
      .def("agentMeanSpeed",
           &dsf::FirstOrderDynamics::agentMeanSpeed,
           dsf::g_docstrings.at("dsf::RoadDynamics::agentMeanSpeed").c_str())
      // .def("streetMeanSpeed", static_cast<double (dsf::FirstOrderDynamics::*)(dsf::Id) const>(&dsf::FirstOrderDynamics::streetMeanSpeed), pybind11::arg("streetId"))
      // .def("streetMeanSpeed", static_cast<dsf::Measurement<double> (dsf::FirstOrderDynamics::*)() const>(&dsf::FirstOrderDynamics::streetMeanSpeed))
      // .def("streetMeanSpeed", static_cast<dsf::Measurement<double> (dsf::FirstOrderDynamics::*)(double, bool) const>(&dsf::FirstOrderDynamics::streetMeanSpeed), pybind11::arg("threshold"), pybind11::arg("above"))
      // .def("streetMeanDensity", &dsf::FirstOrderDynamics::streetMeanDensity, pybind11::arg("normalized") = false)
      // .def("streetMeanFlow", static_cast<dsf::Measurement<double> (dsf::FirstOrderDynamics::*)() const>(&dsf::FirstOrderDynamics::streetMeanFlow))
      // .def("streetMeanFlow", static_cast<dsf::Measurement<double> (dsf::FirstOrderDynamics::*)(double, bool) const>(&dsf::FirstOrderDynamics::streetMeanFlow), pybind11::arg("threshold"), pybind11::arg("above"))
      .def("meanSpireInputFlow",
           &dsf::FirstOrderDynamics::meanSpireInputFlow,
           pybind11::arg("resetValue") = true,
           dsf::g_docstrings.at("dsf::RoadDynamics::meanSpireInputFlow").c_str())
      .def("meanSpireOutputFlow",
           &dsf::FirstOrderDynamics::meanSpireOutputFlow,
           pybind11::arg("resetValue") = true,
           dsf::g_docstrings.at("dsf::RoadDynamics::meanSpireOutputFlow").c_str())
      .def("saveStreetDensities",
           &dsf::FirstOrderDynamics::saveStreetDensities,
           pybind11::arg("filename"),
           pybind11::arg("normalized") = true,
           pybind11::arg("separator") = ';',
           dsf::g_docstrings.at("dsf::RoadDynamics::saveStreetDensities").c_str())
      .def("saveInputStreetCounts",
           &dsf::FirstOrderDynamics::saveInputStreetCounts,
           pybind11::arg("filename"),
           pybind11::arg("reset") = false,
           pybind11::arg("separator") = ';',
           dsf::g_docstrings.at("dsf::RoadDynamics::saveInputStreetCounts").c_str())
      .def("saveOutputStreetCounts",
           &dsf::FirstOrderDynamics::saveOutputStreetCounts,
           pybind11::arg("filename"),
           pybind11::arg("reset") = false,
           pybind11::arg("separator") = ';',
           dsf::g_docstrings.at("dsf::RoadDynamics::saveOutputStreetCounts").c_str())
      .def("saveTravelData",
           &dsf::FirstOrderDynamics::saveTravelData,
           pybind11::arg("filename"),
           pybind11::arg("reset") = false,
           dsf::g_docstrings.at("dsf::RoadDynamics::saveTravelData").c_str())
      .def("saveMacroscopicObservables",
           &dsf::FirstOrderDynamics::saveMacroscopicObservables,
           pybind11::arg("filename"),
           pybind11::arg("separator") = ';',
           dsf::g_docstrings.at("dsf::RoadDynamics::saveMacroscopicObservables").c_str());
}