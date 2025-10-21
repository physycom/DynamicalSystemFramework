#include "./dsf.hpp"

#include "./.docstrings.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>         // Changed to include all stl type casters
#include <pybind11/functional.h>  // For std::function support
#include <pybind11/numpy.h>       // For numpy array support

#include <spdlog/spdlog.h>  // For logging functionality

PYBIND11_MODULE(dsf_cpp, m) {
  m.doc() = "Python bindings for the DSM library";
  m.attr("__version__") = dsf::version();

  // Bind PathWeight enum
  pybind11::enum_<dsf::PathWeight>(m, "PathWeight")
      .value("LENGTH", dsf::PathWeight::LENGTH)
      .value("TRAVELTIME", dsf::PathWeight::TRAVELTIME)
      .value("WEIGHT", dsf::PathWeight::WEIGHT)
      .export_values();

  // Bind TrafficLightOptimization enum
  pybind11::enum_<dsf::TrafficLightOptimization>(m, "TrafficLightOptimization")
      .value("SINGLE_TAIL", dsf::TrafficLightOptimization::SINGLE_TAIL)
      .value("DOUBLE_TAIL", dsf::TrafficLightOptimization::DOUBLE_TAIL)
      .export_values();

  // Bind spdlog log level enum
  pybind11::enum_<spdlog::level::level_enum>(m, "LogLevel")
      .value("TRACE", spdlog::level::trace)
      .value("DEBUG", spdlog::level::debug)
      .value("INFO", spdlog::level::info)
      .value("WARN", spdlog::level::warn)
      .value("ERROR", spdlog::level::err)
      .value("CRITICAL", spdlog::level::critical)
      .value("OFF", spdlog::level::off)
      .export_values();

  // Bind spdlog logging functions
  m.def("set_log_level",
        &spdlog::set_level,
        pybind11::arg("level"),
        "Set the global log level for spdlog");

  m.def("get_log_level", &spdlog::get_level, "Get the current global log level");

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
      .def("nNodes",
           &dsf::RoadNetwork::nNodes,
           dsf::g_docstrings.at("dsf::Network::nNodes").c_str())
      .def("nEdges",
           &dsf::RoadNetwork::nEdges,
           dsf::g_docstrings.at("dsf::Network::nEdges").c_str())
      .def("nCoils",
           &dsf::RoadNetwork::nCoils,
           dsf::g_docstrings.at("dsf::RoadNetwork::nCoils").c_str())
      .def("nIntersections",
           &dsf::RoadNetwork::nIntersections,
           dsf::g_docstrings.at("dsf::RoadNetwork::nIntersections").c_str())
      .def("nRoundabouts",
           &dsf::RoadNetwork::nRoundabouts,
           dsf::g_docstrings.at("dsf::RoadNetwork::nRoundabouts").c_str())
      .def("nTrafficLights",
           &dsf::RoadNetwork::nTrafficLights,
           dsf::g_docstrings.at("dsf::RoadNetwork::nTrafficLights").c_str())
      .def("capacity",
           &dsf::RoadNetwork::capacity,
           dsf::g_docstrings.at("dsf::RoadNetwork::capacity").c_str())
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
      .def(
          "importEdges",
          [](dsf::RoadNetwork& self, const std::string& fileName) {
            self.importEdges(fileName);
          },
          pybind11::arg("fileName"),
          dsf::g_docstrings.at("dsf::RoadNetwork::importEdges").c_str())
      .def(
          "importEdges",
          [](dsf::RoadNetwork& self, std::string const& fileName, char const separator) {
            self.importEdges(fileName, separator);
          },
          pybind11::arg("fileName"),
          pybind11::arg("separator"),
          dsf::g_docstrings.at("dsf::RoadNetwork::importEdges").c_str())
      .def(
          "importEdges",
          [](dsf::RoadNetwork& self,
             std::string const& fileName,
             bool const bCreateInverse) { self.importEdges(fileName, bCreateInverse); },
          pybind11::arg("fileName"),
          pybind11::arg("bCreateInverse"),
          dsf::g_docstrings.at("dsf::RoadNetwork::importEdges").c_str())
      .def(
          "importNodeProperties",
          [](dsf::RoadNetwork& self, std::string const& fileName, char const separator) {
            self.importNodeProperties(fileName, separator);
          },
          pybind11::arg("fileName"),
          pybind11::arg("separator") = ';',
          dsf::g_docstrings.at("dsf::RoadNetwork::importNodeProperties").c_str())
      .def("importTrafficLights",
           &dsf::RoadNetwork::importTrafficLights,
           pybind11::arg("fileName"),
           dsf::g_docstrings.at("dsf::RoadNetwork::importTrafficLights").c_str())
      .def(
          "makeRoundabout",
          [](dsf::RoadNetwork& self, dsf::Id id) -> void { self.makeRoundabout(id); },
          pybind11::arg("id"),
          dsf::g_docstrings.at("dsf::RoadNetwork::makeRoundabout").c_str())
      .def(
          "makeTrafficLight",
          [](dsf::RoadNetwork& self,
             dsf::Id id,
             dsf::Delay const cycleTime,
             dsf::Delay const counter) -> void {
            self.makeTrafficLight(id, cycleTime, counter);
          },
          pybind11::arg("id"),
          pybind11::arg("cycleTime"),
          pybind11::arg("counter"),
          dsf::g_docstrings.at("dsf::RoadNetwork::makeTrafficLight").c_str())
      .def(
          "makeSpireStreet",
          [](dsf::RoadNetwork& self, dsf::Id id) -> void { self.makeSpireStreet(id); },
          pybind11::arg("id"),
          dsf::g_docstrings.at("dsf::RoadNetwork::makeSpireStreet").c_str());

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
      .def(pybind11::init<dsf::RoadNetwork&,
                          bool,
                          std::optional<unsigned int>,
                          double,
                          dsf::PathWeight,
                          std::optional<double>>(),
           pybind11::arg("graph"),
           pybind11::arg("useCache") = false,
           pybind11::arg("seed") = std::nullopt,
           pybind11::arg("alpha") = 0.,
           pybind11::arg("weightFunction") = dsf::PathWeight::TRAVELTIME,
           pybind11::arg("weightThreshold") = std::nullopt,
           dsf::g_docstrings.at("dsf::FirstOrderDynamics::FirstOrderDynamics").c_str())
      // Note: Constructors with std::function parameters are not exposed to avoid stub generation issues
      .def("setInitTime",
           &dsf::FirstOrderDynamics::setInitTime,
           pybind11::arg("timeEpoch"),
           dsf::g_docstrings.at("dsf::Dynamics::setInitTime").c_str())
      .def(
          "setInitTime",
          [](dsf::FirstOrderDynamics& self, pybind11::object datetime_obj) {
            auto const epoch =
                pybind11::cast<std::time_t>(datetime_obj.attr("timestamp")());
            self.setInitTime(epoch);
          },
          pybind11::arg("datetime"),
          dsf::g_docstrings.at("dsf::Dynamics::setInitTime").c_str())
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
            self.setMaxTravelTime(static_cast<std::time_t>(maxTravelTime));
          },
          pybind11::arg("maxTravelTime"),
          dsf::g_docstrings.at("dsf::RoadDynamics::setMaxTravelTime").c_str())
      .def("setErrorProbability",
           &dsf::FirstOrderDynamics::setErrorProbability,
           pybind11::arg("errorProbability"),
           dsf::g_docstrings.at("dsf::RoadDynamics::setErrorProbability").c_str())
      .def("setWeightFunction",
           &dsf::FirstOrderDynamics::setWeightFunction,
           pybind11::arg("weightFunction"),
           pybind11::arg("weightThreshold") = std::nullopt)
      .def(
          "setDestinationNodes",
          [](dsf::FirstOrderDynamics& self,
             const std::vector<dsf::Id>& destinationNodes) {
            self.setDestinationNodes(destinationNodes);
          },
          pybind11::arg("destinationNodes"),
          dsf::g_docstrings.at("dsf::RoadDynamics::setDestinationNodes").c_str())
      .def(
          "setOriginNodes",
          [](dsf::FirstOrderDynamics& self,
             const std::unordered_map<dsf::Id, double>& originNodes) {
            self.setOriginNodes(originNodes);
          },
          pybind11::arg("originNodes"),
          dsf::g_docstrings.at("dsf::RoadDynamics::setOriginNodes").c_str())
      .def(
          "setOriginNodes",
          [](dsf::FirstOrderDynamics& self, pybind11::array_t<dsf::Id> originNodes) {
            // Convert numpy array to vector with equal weights
            auto buf = originNodes.request();
            auto* ptr = static_cast<dsf::Id*>(buf.ptr);
            std::unordered_map<dsf::Id, double> nodeWeights;
            for (size_t i = 0; i < buf.size; ++i) {
              nodeWeights[ptr[i]] = 1.0;  // Equal weight for all nodes
            }
            self.setOriginNodes(nodeWeights);
          },
          pybind11::arg("originNodes"),
          dsf::g_docstrings.at("dsf::RoadDynamics::setOriginNodes").c_str())
      .def(
          "setDestinationNodes",
          [](dsf::FirstOrderDynamics& self, pybind11::array_t<dsf::Id> destinationNodes) {
            // Convert numpy array to vector
            auto buf = destinationNodes.request();
            auto* ptr = static_cast<dsf::Id*>(buf.ptr);
            std::vector<dsf::Id> nodes(ptr, ptr + buf.size);
            self.setDestinationNodes(nodes);
          },
          pybind11::arg("destinationNodes"),
          dsf::g_docstrings.at("dsf::RoadDynamics::setDestinationNodes").c_str())
      .def(
          "setDestinationNodes",
          [](dsf::FirstOrderDynamics& self,
             const std::unordered_map<dsf::Id, double>& destinationNodes) {
            self.setDestinationNodes(destinationNodes);
          },
          pybind11::arg("destinationNodes"),
          dsf::g_docstrings.at("dsf::RoadDynamics::setDestinationNodes").c_str())
      .def("initTurnCounts",
           &dsf::FirstOrderDynamics::initTurnCounts,
           dsf::g_docstrings.at("dsf::RoadDynamics::initTurnCounts").c_str())
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
          [](dsf::FirstOrderDynamics& self, dsf::Size nAgents) {
            self.addAgentsRandomly(nAgents);
          },
          pybind11::arg("nAgents"),
          dsf::g_docstrings.at("dsf::RoadDynamics::addAgentsRandomly").c_str())
      .def(
          "addAgentsRandomly",
          [](dsf::FirstOrderDynamics& self,
             dsf::Size nAgents,
             const std::unordered_map<dsf::Id, double>& src_weights,
             const std::unordered_map<dsf::Id, double>& dst_weights) {
            self.addAgentsRandomly(nAgents, src_weights, dst_weights);
          },
          pybind11::arg("nAgents"),
          pybind11::arg("src_weights"),
          pybind11::arg("dst_weights"),
          dsf::g_docstrings.at("dsf::RoadDynamics::addAgentsRandomly").c_str())
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
      .def("nAgents",
           &dsf::FirstOrderDynamics::nAgents,
           dsf::g_docstrings.at("dsf::RoadDynamics::nAgents").c_str())
      .def("time",
           &dsf::FirstOrderDynamics::time,
           dsf::g_docstrings.at("dsf::Dynamics::time").c_str())
      .def("time_step",
           &dsf::FirstOrderDynamics::time_step,
           dsf::g_docstrings.at("dsf::Dynamics::time_step").c_str())
      .def("datetime",
           &dsf::FirstOrderDynamics::strDateTime,
           dsf::g_docstrings.at("dsf::Dynamics::strDateTime").c_str())
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
      .def(
          "turnCounts",
          [](const dsf::FirstOrderDynamics& self) {
            // Convert C++ unordered_map<Id, unordered_map<Id, size_t>> to Python dict of dicts
            pybind11::dict py_result;
            for (const auto& [from_id, inner_map] : self.turnCounts()) {
              pybind11::dict py_inner;
              for (const auto& [to_id, count] : inner_map) {
                py_inner[pybind11::int_(to_id)] = pybind11::int_(count);
              }
              py_result[pybind11::int_(from_id)] = py_inner;
            }
            return py_result;
          },
          dsf::g_docstrings.at("dsf::RoadDynamics::turnCounts").c_str())
      .def(
          "normalizedTurnCounts",
          [](const dsf::FirstOrderDynamics& self) {
            // Convert C++ unordered_map<Id, unordered_map<Id, size_t>> to Python dict of dicts
            pybind11::dict py_result;
            for (const auto& [from_id, inner_map] : self.normalizedTurnCounts()) {
              pybind11::dict py_inner;
              for (const auto& [to_id, count] : inner_map) {
                py_inner[pybind11::int_(to_id)] = pybind11::float_(count);
              }
              py_result[pybind11::int_(from_id)] = py_inner;
            }
            return py_result;
          },
          dsf::g_docstrings.at("dsf::RoadDynamics::normalizedTurnCounts").c_str())
      //  .def("turnProbabilities",
      //       &dsf::FirstOrderDynamics::turnProbabilities,
      //       pybind11::arg("reset") = true)
      //  .def("turnMapping",
      //       &dsf::FirstOrderDynamics::turnMapping,
      //       pybind11::return_value_policy::reference_internal)
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