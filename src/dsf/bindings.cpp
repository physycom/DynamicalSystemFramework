#include "dsf.hpp"

#include ".docstrings.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>         // Changed to include all stl type casters
#include <pybind11/functional.h>  // For std::function support
#include <pybind11/numpy.h>       // For numpy array support

#include <spdlog/spdlog.h>  // For logging functionality

PYBIND11_MODULE(dsf_cpp, m) {
  m.doc() = "Python bindings for the DSF library";
  m.attr("__version__") = dsf::version();

  // Create mobility submodule
  auto mobility = m.def_submodule("mobility",
                                  "Bindings for mobility-related classes and functions, "
                                  "under the dsf::mobility C++ namespace.");
  auto mdt = m.def_submodule("mdt",
                             "Bindings for movement data tools (MDT) related classes and "
                             "functions, under the dsf::mdt C++ namespace.");

  // Bind PathWeight enum
  pybind11::enum_<dsf::PathWeight>(mobility, "PathWeight")
      .value("LENGTH", dsf::PathWeight::LENGTH)
      .value("TRAVELTIME", dsf::PathWeight::TRAVELTIME)
      .value("WEIGHT", dsf::PathWeight::WEIGHT)
      .export_values();

  // Bind TrafficLightOptimization enum
  pybind11::enum_<dsf::TrafficLightOptimization>(mobility, "TrafficLightOptimization")
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

  // Bind Measurement to main module (can be used across different contexts)
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

  // Bind AdjacencyMatrix to main module (general graph structure)
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

  // Bind mobility-related classes to mobility submodule
  pybind11::class_<dsf::mobility::RoadNetwork>(mobility, "RoadNetwork")
      .def(pybind11::init<>(),
           dsf::g_docstrings.at("dsf::mobility::RoadNetwork::RoadNetwork").c_str())
      .def(pybind11::init<const dsf::AdjacencyMatrix&>(),
           dsf::g_docstrings.at("dsf::mobility::RoadNetwork::RoadNetwork").c_str())
      .def("nNodes",
           &dsf::mobility::RoadNetwork::nNodes,
           dsf::g_docstrings.at("dsf::Network::nNodes").c_str())
      .def("nEdges",
           &dsf::mobility::RoadNetwork::nEdges,
           dsf::g_docstrings.at("dsf::Network::nEdges").c_str())
      .def("nCoils",
           &dsf::mobility::RoadNetwork::nCoils,
           dsf::g_docstrings.at("dsf::mobility::RoadNetwork::nCoils").c_str())
      .def("nIntersections",
           &dsf::mobility::RoadNetwork::nIntersections,
           dsf::g_docstrings.at("dsf::mobility::RoadNetwork::nIntersections").c_str())
      .def("nRoundabouts",
           &dsf::mobility::RoadNetwork::nRoundabouts,
           dsf::g_docstrings.at("dsf::mobility::RoadNetwork::nRoundabouts").c_str())
      .def("nTrafficLights",
           &dsf::mobility::RoadNetwork::nTrafficLights,
           dsf::g_docstrings.at("dsf::mobility::RoadNetwork::nTrafficLights").c_str())
      .def("capacity",
           &dsf::mobility::RoadNetwork::capacity,
           dsf::g_docstrings.at("dsf::mobility::RoadNetwork::capacity").c_str())
      .def(
          "adjustNodeCapacities",
          &dsf::mobility::RoadNetwork::adjustNodeCapacities,
          dsf::g_docstrings.at("dsf::mobility::RoadNetwork::adjustNodeCapacities").c_str())
      .def("initTrafficLights",
           &dsf::mobility::RoadNetwork::initTrafficLights,
           pybind11::arg("minGreenTime") = 30,
           dsf::g_docstrings.at("dsf::mobility::RoadNetwork::initTrafficLights").c_str())
      .def("autoMapStreetLanes",
           &dsf::mobility::RoadNetwork::autoMapStreetLanes,
           dsf::g_docstrings.at("dsf::mobility::RoadNetwork::autoMapStreetLanes").c_str())
      .def(
          "importEdges",
          [](dsf::mobility::RoadNetwork& self, const std::string& fileName) {
            self.importEdges(fileName);
          },
          pybind11::arg("fileName"),
          dsf::g_docstrings.at("dsf::mobility::RoadNetwork::importEdges").c_str())
      .def(
          "importEdges",
          [](dsf::mobility::RoadNetwork& self,
             std::string const& fileName,
             char const separator) { self.importEdges(fileName, separator); },
          pybind11::arg("fileName"),
          pybind11::arg("separator"),
          dsf::g_docstrings.at("dsf::mobility::RoadNetwork::importEdges").c_str())
      .def(
          "importEdges",
          [](dsf::mobility::RoadNetwork& self,
             std::string const& fileName,
             bool const bCreateInverse) { self.importEdges(fileName, bCreateInverse); },
          pybind11::arg("fileName"),
          pybind11::arg("bCreateInverse"),
          dsf::g_docstrings.at("dsf::mobility::RoadNetwork::importEdges").c_str())
      .def(
          "importNodeProperties",
          [](dsf::mobility::RoadNetwork& self,
             std::string const& fileName,
             char const separator) { self.importNodeProperties(fileName, separator); },
          pybind11::arg("fileName"),
          pybind11::arg("separator") = ';',
          dsf::g_docstrings.at("dsf::mobility::RoadNetwork::importNodeProperties").c_str())
      .def(
          "importTrafficLights",
          &dsf::mobility::RoadNetwork::importTrafficLights,
          pybind11::arg("fileName"),
          dsf::g_docstrings.at("dsf::mobility::RoadNetwork::importTrafficLights").c_str())
      .def(
          "makeRoundabout",
          [](dsf::mobility::RoadNetwork& self, dsf::Id id) -> void {
            self.makeRoundabout(id);
          },
          pybind11::arg("id"),
          dsf::g_docstrings.at("dsf::mobility::RoadNetwork::makeRoundabout").c_str())
      .def(
          "makeTrafficLight",
          [](dsf::mobility::RoadNetwork& self,
             dsf::Id id,
             dsf::Delay const cycleTime,
             dsf::Delay const counter) -> void {
            self.makeTrafficLight(id, cycleTime, counter);
          },
          pybind11::arg("id"),
          pybind11::arg("cycleTime"),
          pybind11::arg("counter"),
          dsf::g_docstrings.at("dsf::mobility::RoadNetwork::makeTrafficLight").c_str())
      .def("addCoil",
           &dsf::mobility::RoadNetwork::addCoil,
           pybind11::arg("streetId"),
           pybind11::arg("name") = std::string(),
           dsf::g_docstrings.at("dsf::mobility::RoadNetwork::addCoil").c_str())
      .def(
          "shortestPath",
          [](const dsf::mobility::RoadNetwork& self,
             dsf::Id sourceId,
             dsf::Id targetId,
             dsf::PathWeight weightFunction,
             double threshold) {
            return self.shortestPath(
                sourceId,
                targetId,
                [weightFunction](const std::unique_ptr<dsf::mobility::Street>& street) {
                  switch (weightFunction) {
                    case dsf::PathWeight::LENGTH:
                      return street->length();
                    case dsf::PathWeight::TRAVELTIME:
                      return street->length() / street->maxSpeed();
                    case dsf::PathWeight::WEIGHT:
                      return street->weight();
                    default:
                      return street->length() / street->maxSpeed();
                  }
                },
                threshold);
          },
          pybind11::arg("sourceId"),
          pybind11::arg("targetId"),
          pybind11::arg("weightFunction") = dsf::PathWeight::TRAVELTIME,
          pybind11::arg("threshold") = 1e-9,
          "Find the shortest path between two nodes using Dijkstra's algorithm.\n\n"
          "Args:\n"
          "    sourceId (int): The id of the source node\n"
          "    targetId (int): The id of the target node\n"
          "    weightFunction (PathWeight): The weight function to use (LENGTH, "
          "TRAVELTIME, or WEIGHT)\n"
          "    threshold (float): A threshold value to consider alternative paths\n\n"
          "Returns:\n"
          "    PathCollection: A map where each key is a node id and the value is a "
          "vector of next hop node ids toward the target");

  pybind11::class_<dsf::mobility::PathCollection>(mobility, "PathCollection")
      .def(pybind11::init<>(), "Create an empty PathCollection")
      .def(
          "__getitem__",
          [](const dsf::mobility::PathCollection& self, dsf::Id key) {
            auto it = self.find(key);
            if (it == self.end()) {
              throw pybind11::key_error("Key not found");
            }
            return it->second;
          },
          pybind11::arg("key"),
          "Get the next hops for a given node id")
      .def(
          "__setitem__",
          [](dsf::mobility::PathCollection& self,
             dsf::Id key,
             std::vector<dsf::Id> value) { self[key] = value; },
          pybind11::arg("key"),
          pybind11::arg("value"),
          "Set the next hops for a given node id")
      .def(
          "__contains__",
          [](const dsf::mobility::PathCollection& self, dsf::Id key) {
            return self.find(key) != self.end();
          },
          pybind11::arg("key"),
          "Check if a node id exists in the collection")
      .def(
          "__len__",
          [](const dsf::mobility::PathCollection& self) { return self.size(); },
          "Get the number of nodes in the collection")
      .def(
          "keys",
          [](const dsf::mobility::PathCollection& self) {
            std::vector<dsf::Id> keys;
            keys.reserve(self.size());
            for (const auto& [key, _] : self) {
              keys.push_back(key);
            }
            return keys;
          },
          "Get all node ids in the collection")
      .def(
          "items",
          [](const dsf::mobility::PathCollection& self) {
            pybind11::dict items;
            for (const auto& [key, value] : self) {
              items[pybind11::int_(key)] = pybind11::cast(value);
            }
            return items;
          },
          "Get all items (node id, next hops) in the collection")
      .def("explode",
           &dsf::mobility::PathCollection::explode,
           pybind11::arg("sourceId"),
           pybind11::arg("targetId"),
           dsf::g_docstrings.at("dsf::mobility::PathCollection::explode").c_str());

  pybind11::class_<dsf::mobility::Itinerary>(mobility, "Itinerary")
      .def(pybind11::init<dsf::Id, dsf::Id>(),
           pybind11::arg("id"),
           pybind11::arg("destination"),
           dsf::g_docstrings.at("dsf::mobility::Itinerary::Itinerary").c_str())
      .def("setPath",
           &dsf::mobility::Itinerary::setPath,
           pybind11::arg("path"),
           dsf::g_docstrings.at("dsf::mobility::Itinerary::setPath").c_str())
      .def("id",
           &dsf::mobility::Itinerary::id,
           dsf::g_docstrings.at("dsf::mobility::Itinerary::id").c_str())
      .def("destination",
           &dsf::mobility::Itinerary::destination,
           dsf::g_docstrings.at("dsf::mobility::Itinerary::destination").c_str());
  // .def("path", &dsf::mobility::Itinerary::path, pybind11::return_value_policy::reference_internal);

  pybind11::class_<dsf::mobility::FirstOrderDynamics>(mobility, "Dynamics")
      //     // Constructors are not directly exposed due to the template nature and complexity.
      //     // Users should use derived classes like FirstOrderDynamics.
      .def(pybind11::init<dsf::mobility::RoadNetwork&,
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
           dsf::g_docstrings.at("dsf::mobility::FirstOrderDynamics::FirstOrderDynamics")
               .c_str())
      // Note: Constructors with std::function parameters are not exposed to avoid stub generation issues
      .def("setName",
           &dsf::mobility::FirstOrderDynamics::setName,
           pybind11::arg("name"),
           dsf::g_docstrings.at("dsf::Dynamics::setName").c_str())
      .def("setInitTime",
           &dsf::mobility::FirstOrderDynamics::setInitTime,
           pybind11::arg("timeEpoch"),
           dsf::g_docstrings.at("dsf::Dynamics::setInitTime").c_str())
      .def(
          "setInitTime",
          [](dsf::mobility::FirstOrderDynamics& self, pybind11::object datetime_obj) {
            auto const epoch =
                pybind11::cast<std::time_t>(datetime_obj.attr("timestamp")());
            self.setInitTime(epoch);
          },
          pybind11::arg("datetime"),
          dsf::g_docstrings.at("dsf::Dynamics::setInitTime").c_str())
      .def(
          "setForcePriorities",
          &dsf::mobility::FirstOrderDynamics::setForcePriorities,
          pybind11::arg("forcePriorities"),
          dsf::g_docstrings.at("dsf::mobility::RoadDynamics::setForcePriorities").c_str())
      .def(
          "setDataUpdatePeriod",
          [](dsf::mobility::FirstOrderDynamics& self, int dataUpdatePeriod) {
            self.setDataUpdatePeriod(static_cast<dsf::Delay>(dataUpdatePeriod));
          },
          pybind11::arg("dataUpdatePeriod"),
          dsf::g_docstrings.at("dsf::mobility::RoadDynamics::setDataUpdatePeriod").c_str())
      .def("setMaxDistance",
           &dsf::mobility::FirstOrderDynamics::setMaxDistance,
           pybind11::arg("maxDistance"),
           dsf::g_docstrings.at("dsf::mobility::RoadDynamics::setMaxDistance").c_str())
      .def(
          "setMaxTravelTime",
          [](dsf::mobility::FirstOrderDynamics& self, uint64_t maxTravelTime) {
            self.setMaxTravelTime(static_cast<std::time_t>(maxTravelTime));
          },
          pybind11::arg("maxTravelTime"),
          dsf::g_docstrings.at("dsf::mobility::RoadDynamics::setMaxTravelTime").c_str())
      .def(
          "setErrorProbability",
          &dsf::mobility::FirstOrderDynamics::setErrorProbability,
          pybind11::arg("errorProbability"),
          dsf::g_docstrings.at("dsf::mobility::RoadDynamics::setErrorProbability").c_str())
      .def("setWeightFunction",
           &dsf::mobility::FirstOrderDynamics::setWeightFunction,
           pybind11::arg("weightFunction"),
           pybind11::arg("weightThreshold") = std::nullopt)
      .def(
          "killStagnantAgents",
          &dsf::mobility::FirstOrderDynamics::killStagnantAgents,
          pybind11::arg("timeToleranceFactor") = 3.,
          dsf::g_docstrings.at("dsf::mobility::RoadDynamics::killStagnantAgents").c_str())
      .def(
          "setDestinationNodes",
          [](dsf::mobility::FirstOrderDynamics& self,
             const std::vector<dsf::Id>& destinationNodes) {
            self.setDestinationNodes(destinationNodes);
          },
          pybind11::arg("destinationNodes"),
          dsf::g_docstrings.at("dsf::mobility::RoadDynamics::setDestinationNodes").c_str())
      .def(
          "setOriginNodes",
          [](dsf::mobility::FirstOrderDynamics& self,
             const std::unordered_map<dsf::Id, double>& originNodes) {
            self.setOriginNodes(originNodes);
          },
          pybind11::arg("originNodes"),
          dsf::g_docstrings.at("dsf::mobility::RoadDynamics::setOriginNodes").c_str())
      .def(
          "setOriginNodes",
          [](dsf::mobility::FirstOrderDynamics& self,
             pybind11::array_t<dsf::Id> originNodes) {
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
          dsf::g_docstrings.at("dsf::mobility::RoadDynamics::setOriginNodes").c_str())
      .def(
          "setDestinationNodes",
          [](dsf::mobility::FirstOrderDynamics& self,
             pybind11::array_t<dsf::Id> destinationNodes) {
            // Convert numpy array to vector
            auto buf = destinationNodes.request();
            auto* ptr = static_cast<dsf::Id*>(buf.ptr);
            std::vector<dsf::Id> nodes(ptr, ptr + buf.size);
            self.setDestinationNodes(nodes);
          },
          pybind11::arg("destinationNodes"),
          dsf::g_docstrings.at("dsf::mobility::RoadDynamics::setDestinationNodes").c_str())
      .def(
          "setDestinationNodes",
          [](dsf::mobility::FirstOrderDynamics& self,
             const std::unordered_map<dsf::Id, double>& destinationNodes) {
            self.setDestinationNodes(destinationNodes);
          },
          pybind11::arg("destinationNodes"),
          dsf::g_docstrings.at("dsf::mobility::RoadDynamics::setDestinationNodes").c_str())
      .def("initTurnCounts",
           &dsf::mobility::FirstOrderDynamics::initTurnCounts,
           dsf::g_docstrings.at("dsf::mobility::RoadDynamics::initTurnCounts").c_str())
      .def("updatePaths",
           &dsf::mobility::FirstOrderDynamics::updatePaths,
           dsf::g_docstrings.at("dsf::mobility::RoadDynamics::updatePaths").c_str())
      .def(
          "addAgentsUniformly",
          &dsf::mobility::FirstOrderDynamics::addAgentsUniformly,
          pybind11::arg("nAgents"),
          pybind11::arg("itineraryId") = std::nullopt,
          dsf::g_docstrings.at("dsf::mobility::RoadDynamics::addAgentsUniformly").c_str())
      .def(
          "addAgentsRandomly",
          [](dsf::mobility::FirstOrderDynamics& self, dsf::Size nAgents) {
            self.addAgentsRandomly(nAgents);
          },
          pybind11::arg("nAgents"),
          dsf::g_docstrings.at("dsf::mobility::RoadDynamics::addAgentsRandomly").c_str())
      .def(
          "addAgentsRandomly",
          [](dsf::mobility::FirstOrderDynamics& self,
             dsf::Size nAgents,
             const std::unordered_map<dsf::Id, double>& src_weights,
             const std::unordered_map<dsf::Id, double>& dst_weights) {
            self.addAgentsRandomly(nAgents, src_weights, dst_weights);
          },
          pybind11::arg("nAgents"),
          pybind11::arg("src_weights"),
          pybind11::arg("dst_weights"),
          dsf::g_docstrings.at("dsf::mobility::RoadDynamics::addAgentsRandomly").c_str())
      .def("evolve",
           &dsf::mobility::FirstOrderDynamics::evolve,
           pybind11::arg("reinsert_agents") = false,
           dsf::g_docstrings.at("dsf::mobility::RoadDynamics::evolve").c_str())
      .def("optimizeTrafficLights",
           &dsf::mobility::FirstOrderDynamics::optimizeTrafficLights,
           pybind11::arg("optimizationType") = dsf::TrafficLightOptimization::DOUBLE_TAIL,
           pybind11::arg("logFile") = "",
           pybind11::arg("threshold") = 0.,
           pybind11::arg("ratio") = 1.3,
           dsf::g_docstrings.at("dsf::mobility::RoadDynamics::optimizeTrafficLights")
               .c_str())
      .def("nAgents",
           &dsf::mobility::FirstOrderDynamics::nAgents,
           dsf::g_docstrings.at("dsf::mobility::RoadDynamics::nAgents").c_str())
      .def("time",
           &dsf::mobility::FirstOrderDynamics::time,
           dsf::g_docstrings.at("dsf::Dynamics::time").c_str())
      .def("time_step",
           &dsf::mobility::FirstOrderDynamics::time_step,
           dsf::g_docstrings.at("dsf::Dynamics::time_step").c_str())
      .def("datetime",
           &dsf::mobility::FirstOrderDynamics::strDateTime,
           dsf::g_docstrings.at("dsf::Dynamics::strDateTime").c_str())
      .def("meanTravelTime",
           &dsf::mobility::FirstOrderDynamics::meanTravelTime,
           pybind11::arg("clearData") = false,
           dsf::g_docstrings.at("dsf::mobility::RoadDynamics::meanTravelTime").c_str())
      .def(
          "meanTravelDistance",
          &dsf::mobility::FirstOrderDynamics::meanTravelDistance,
          pybind11::arg("clearData") = false,
          dsf::g_docstrings.at("dsf::mobility::RoadDynamics::meanTravelDistance").c_str())
      .def("meanTravelSpeed",
           &dsf::mobility::FirstOrderDynamics::meanTravelSpeed,
           pybind11::arg("clearData") = false,
           dsf::g_docstrings.at("dsf::mobility::RoadDynamics::meanTravelSpeed").c_str())
      .def(
          "turnCounts",
          [](const dsf::mobility::FirstOrderDynamics& self) {
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
          dsf::g_docstrings.at("dsf::mobility::RoadDynamics::turnCounts").c_str())
      .def(
          "normalizedTurnCounts",
          [](const dsf::mobility::FirstOrderDynamics& self) {
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
          dsf::g_docstrings.at("dsf::mobility::RoadDynamics::normalizedTurnCounts")
              .c_str())
      .def(
          "saveStreetDensities",
          &dsf::mobility::FirstOrderDynamics::saveStreetDensities,
          pybind11::arg("filename"),
          pybind11::arg("normalized") = true,
          pybind11::arg("separator") = ';',
          dsf::g_docstrings.at("dsf::mobility::RoadDynamics::saveStreetDensities").c_str())
      .def("saveCoilCounts",
           &dsf::mobility::FirstOrderDynamics::saveCoilCounts,
           pybind11::arg("filename"),
           pybind11::arg("reset") = false,
           pybind11::arg("separator") = ';',
           dsf::g_docstrings.at("dsf::mobility::RoadDynamics::saveCoilCounts").c_str())
      .def("saveTravelData",
           &dsf::mobility::FirstOrderDynamics::saveTravelData,
           pybind11::arg("filename"),
           pybind11::arg("reset") = false,
           dsf::g_docstrings.at("dsf::mobility::RoadDynamics::saveTravelData").c_str())
      .def("saveMacroscopicObservables",
           &dsf::mobility::FirstOrderDynamics::saveMacroscopicObservables,
           pybind11::arg("filename"),
           pybind11::arg("separator") = ';',
           dsf::g_docstrings.at("dsf::mobility::RoadDynamics::saveMacroscopicObservables")
               .c_str());

  // Bind TrajectoryCollection class to mdt submodule
  pybind11::class_<dsf::mdt::TrajectoryCollection>(mdt, "TrajectoryCollection")
      .def(pybind11::init<std::string const&,
                          std::unordered_map<std::string, std::string> const&,
                          char const,
                          std::array<double, 4> const&>(),
           pybind11::arg("fileName"),
           pybind11::arg("column_mapping") =
               std::unordered_map<std::string, std::string>{},
           pybind11::arg("separator") = ';',
           pybind11::arg("bbox") = std::array<double, 4>{},
           dsf::g_docstrings.at("dsf::mdt::TrajectoryCollection::TrajectoryCollection")
               .c_str())
      .def(
          pybind11::init([](pybind11::object df) {
            pybind11::object columns = df.attr("columns");
            pybind11::array arr = df.attr("to_numpy")();
            // Expect a 2D numpy array (rows x cols) and an iterable of column names
            auto info = arr.request();
            if (info.ndim != 2) {
              throw std::runtime_error(
                  "TrajectoryCollection constructor expects a 2D numpy array from "
                  "df.to_numpy()");
            }
            std::size_t n_rows = static_cast<std::size_t>(info.shape[0]);
            std::size_t n_cols = static_cast<std::size_t>(info.shape[1]);

            // Collect column names
            std::vector<std::string> colnames;
            for (auto item : columns) {
              colnames.push_back(pybind11::str(item));
            }

            // Build unordered_map<string, vector<string>> where each key is a column name
            std::unordered_map<std::string,
                               std::variant<std::vector<dsf::Id>,
                                            std::vector<std::time_t>,
                                            std::vector<double>>>
                dataframe;
            dataframe.reserve(n_cols);

            // Columns should be uid timestamp lat lon
            dataframe["uid"] = std::vector<dsf::Id>();
            std::get<std::vector<dsf::Id>>(dataframe.at("uid")).reserve(n_rows);
            dataframe["timestamp"] = std::vector<std::time_t>();
            std::get<std::vector<std::time_t>>(dataframe.at("timestamp")).reserve(n_rows);
            dataframe["lat"] = std::vector<double>();
            std::get<std::vector<double>>(dataframe.at("lat")).reserve(n_rows);
            dataframe["lon"] = std::vector<double>();
            std::get<std::vector<double>>(dataframe.at("lon")).reserve(n_rows);

            for (auto const& colname : colnames) {
              if (colname == "uid") {
                for (std::size_t i = 0; i < n_rows; ++i) {
                  pybind11::object cell = arr[pybind11::make_tuple(i, 0)];
                  std::get<std::vector<dsf::Id>>(dataframe.at("uid"))
                      .push_back(static_cast<dsf::Id>(pybind11::cast<double>(cell)));
                }
              } else if (colname == "timestamp") {
                for (std::size_t i = 0; i < n_rows; ++i) {
                  pybind11::object cell = arr[pybind11::make_tuple(i, 1)];
                  std::get<std::vector<std::time_t>>(dataframe.at("timestamp"))
                      .push_back(static_cast<std::time_t>(pybind11::cast<double>(cell)));
                }
              } else if (colname == "lat") {
                for (std::size_t i = 0; i < n_rows; ++i) {
                  pybind11::object cell = arr[pybind11::make_tuple(i, 2)];
                  std::get<std::vector<double>>(dataframe.at("lat"))
                      .push_back(pybind11::cast<double>(cell));
                }
              } else if (colname == "lon") {
                for (std::size_t i = 0; i < n_rows; ++i) {
                  pybind11::object cell = arr[pybind11::make_tuple(i, 3)];
                  std::get<std::vector<double>>(dataframe.at("lon"))
                      .push_back(pybind11::cast<double>(cell));
                }
              }
            }

            return new dsf::mdt::TrajectoryCollection(std::move(dataframe));
          }),
          pybind11::arg("df"),
          // Write this docstring manually as it is not in g_docstrings
          "Constructor that builds a TrajectoryCollection from a pandas or polars "
          "DataFrame.\n\nArgs:\n\tdf (pandas.DataFrame | polars.DataFrame): Input "
          "DataFrame. Must contain the following columns:\n\t\t'uid' (identifier), "
          "'timestamp' (epoch seconds), 'lat' (latitude),\n\t\t'lon' (longitude). The "
          "constructor will call ``df.columns`` and\n\t\t``df.to_numpy()`` internally. "
          "All cell values are converted to strings\n\t\twhen building the underlying "
          "C++ data structure.\n\nReturns:\n\tdsf.mdt.TrajectoryCollection: A new "
          "TrajectoryCollection constructed from\n\tthe provided DataFrame.")
      .def("filter",
           &dsf::mdt::TrajectoryCollection::filter,
           pybind11::arg("cluster_radius_km"),
           pybind11::arg("max_speed_kph") = 150.0,
           pybind11::arg("min_points_per_trajectory") = 2,
           pybind11::arg("min_duration_min") = pybind11::none(),
           dsf::g_docstrings.at("dsf::mdt::TrajectoryCollection::filter").c_str())
      .def("to_csv",
           &dsf::mdt::TrajectoryCollection::to_csv,
           pybind11::arg("fileName"),
           pybind11::arg("sep") = ';',
           dsf::g_docstrings.at("dsf::mdt::TrajectoryCollection::to_csv").c_str())
      .def(
          "to_pandas",
          [](const dsf::mdt::TrajectoryCollection& self) {
            // Convert the internal data to a pandas DataFrame
            pybind11::module_ pd = pybind11::module_::import("pandas");
            pybind11::dict data_dict;

            // Prepare columns
            std::vector<dsf::Id> uids;
            std::vector<std::size_t> trajectoryIds;
            std::vector<double> lons;
            std::vector<double> lats;
            std::vector<std::time_t> timestamps_in;
            std::vector<std::time_t> timestamps_out;

            for (auto const& [uid, trajectories] : self.trajectories()) {
              std::size_t trajIdx = 0;
              for (auto const& trajectory : trajectories) {
                for (auto const& cluster : trajectory.points()) {
                  auto const centroid = cluster.centroid();
                  uids.push_back(uid);
                  trajectoryIds.push_back(trajIdx);
                  lons.push_back(centroid.x());
                  lats.push_back(centroid.y());
                  timestamps_in.push_back(cluster.firstTimestamp());
                  timestamps_out.push_back(cluster.lastTimestamp());
                }
                ++trajIdx;
              }
            }

            data_dict["uid"] = pybind11::array(uids.size(), uids.data());
            data_dict["trajectory_id"] =
                pybind11::array(trajectoryIds.size(), trajectoryIds.data());

            data_dict["lon"] = pybind11::array(lons.size(), lons.data());
            data_dict["lat"] = pybind11::array(lats.size(), lats.data());
            data_dict["timestamp_in"] =
                pybind11::array(timestamps_in.size(), timestamps_in.data());
            data_dict["timestamp_out"] =
                pybind11::array(timestamps_out.size(), timestamps_out.data());

            return pd.attr("DataFrame")(data_dict);
          },
          "Convert the TrajectoryCollection to a pandas DataFrame.\n\nReturns:\n\tpandas."
          "DataFrame: DataFrame containing the trajectory data with columns 'uid', "
          "'trajectory_id', 'lon', 'lat', 'timestamp_in', and 'timestamp_out'.")
      .def(
          "to_polars",
          [](const dsf::mdt::TrajectoryCollection& self) {
            // Convert the internal data to a polars DataFrame
            pybind11::module_ pl = pybind11::module_::import("polars");
            pybind11::dict data_dict;

            // Prepare columns
            std::vector<dsf::Id> uids;
            std::vector<std::size_t> trajectoryIds;
            std::vector<double> lons;
            std::vector<double> lats;
            std::vector<std::time_t> timestamps_in;
            std::vector<std::time_t> timestamps_out;

            for (auto const& [uid, trajectories] : self.trajectories()) {
              std::size_t trajIdx = 0;
              for (auto const& trajectory : trajectories) {
                for (auto const& cluster : trajectory.points()) {
                  auto const centroid = cluster.centroid();
                  uids.push_back(uid);
                  trajectoryIds.push_back(trajIdx);
                  lons.push_back(centroid.x());
                  lats.push_back(centroid.y());
                  timestamps_in.push_back(cluster.firstTimestamp());
                  timestamps_out.push_back(cluster.lastTimestamp());
                }
                ++trajIdx;
              }
            }

            data_dict["uid"] = pybind11::array(uids.size(), uids.data());
            data_dict["trajectory_id"] =
                pybind11::array(trajectoryIds.size(), trajectoryIds.data());

            data_dict["lon"] = pybind11::array(lons.size(), lons.data());
            data_dict["lat"] = pybind11::array(lats.size(), lats.data());
            data_dict["timestamp_in"] =
                pybind11::array(timestamps_in.size(), timestamps_in.data());
            data_dict["timestamp_out"] =
                pybind11::array(timestamps_out.size(), timestamps_out.data());

            return pl.attr("DataFrame")(data_dict);
          },
          "Convert the TrajectoryCollection to a polars DataFrame.\n\nReturns:\n\tpolars."
          "DataFrame: DataFrame containing the trajectory data with columns 'uid', "
          "'trajectory_id', 'lon', 'lat', 'timestamp_in', and 'timestamp_out'.");
}