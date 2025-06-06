#include "./headers/AdjacencyMatrix.hpp"
#include "./headers/RoadNetwork.hpp"
#include "./headers/FirstOrderDynamics.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>  // Changed to include all stl type casters

PYBIND11_MODULE(dsf, m) {
  m.doc() = "Python bindings for the DSM library";

  pybind11::class_<dsf::AdjacencyMatrix>(m, "AdjacencyMatrix")
      .def(pybind11::init<>())
      .def(pybind11::init<std::string const&>(),
           pybind11::arg("fileName"))  // Added constructor
      .def("n", &dsf::AdjacencyMatrix::n)
      .def("size", &dsf::AdjacencyMatrix::size)
      .def("empty", &dsf::AdjacencyMatrix::empty)  // Added empty
      .def("getRow", &dsf::AdjacencyMatrix::getRow)
      .def("getCol", &dsf::AdjacencyMatrix::getCol)  // Added getCol
      .def("operator()",
           [](const dsf::AdjacencyMatrix& self, dsf::Id i, dsf::Id j) {
             return self(i, j);
           })
      .def("insert", &dsf::AdjacencyMatrix::insert)      // Added insert
      .def("contains", &dsf::AdjacencyMatrix::contains)  // Added contains
      .def("elements", &dsf::AdjacencyMatrix::elements)  // Added elements
      .def("clear", &dsf::AdjacencyMatrix::clear)
      .def("clearRow", &dsf::AdjacencyMatrix::clearRow)  // Added clearRow
      .def("clearCol", &dsf::AdjacencyMatrix::clearCol)  // Added clearCol
      .def("getInDegreeVector",
           &dsf::AdjacencyMatrix::getInDegreeVector)  // Added getInDegreeVector
      .def("getOutDegreeVector",
           &dsf::AdjacencyMatrix::getOutDegreeVector)  // Added getOutDegreeVector
      .def("read", &dsf::AdjacencyMatrix::read, pybind11::arg("fileName"))   // Added read
      .def("save", &dsf::AdjacencyMatrix::save, pybind11::arg("fileName"));  // Added save

  pybind11::class_<dsf::RoadNetwork>(m, "RoadNetwork")
      .def(pybind11::init<>())
      .def(pybind11::init<const dsf::AdjacencyMatrix&>())
      .def("buildAdj", &dsf::RoadNetwork::buildAdj)
      .def("adjustNodeCapacities", &dsf::RoadNetwork::adjustNodeCapacities)
      .def("initTrafficLights",
           &dsf::RoadNetwork::initTrafficLights,
           pybind11::arg("minGreenTime") = 30)
      .def("autoMapStreetLanes", &dsf::RoadNetwork::autoMapStreetLanes)
      .def("importMatrix",
           &dsf::RoadNetwork::importMatrix,
           pybind11::arg("fileName"),
           pybind11::arg("isAdj") = true,
           pybind11::arg("defaultSpeed") = 13.8888888889)
      .def("importCoordinates",
           &dsf::RoadNetwork::importCoordinates,
           pybind11::arg("fileName"))
      .def("importOSMNodes", &dsf::RoadNetwork::importOSMNodes, pybind11::arg("fileName"))
      .def("importOSMEdges", &dsf::RoadNetwork::importOSMEdges, pybind11::arg("fileName"))
      .def("importTrafficLights",
           &dsf::RoadNetwork::importTrafficLights,
           pybind11::arg("fileName"))
      .def("exportNodes", &dsf::RoadNetwork::exportNodes, pybind11::arg("fileName"))
      .def("exportEdges", &dsf::RoadNetwork::exportEdges, pybind11::arg("fileName"));

  pybind11::class_<dsf::Itinerary>(m, "Itinerary")
      .def(pybind11::init<dsf::Id, dsf::Id>(),
           pybind11::arg("id"),
           pybind11::arg("destination"))
      .def("setPath", &dsf::Itinerary::setPath, pybind11::arg("path"))
      .def("id", &dsf::Itinerary::id)
      .def("destination", &dsf::Itinerary::destination);
  // .def("path", &dsf::Itinerary::path, pybind11::return_value_policy::reference_internal);

  pybind11::class_<dsf::FirstOrderDynamics>(m, "Dynamics")
      //     // Constructors are not directly exposed due to the template nature and complexity.
      //     // Users should use derived classes like FirstOrderDynamics.
      .def(pybind11::init<dsf::RoadNetwork&>(), pybind11::arg("graph"))
      .def(pybind11::init<dsf::RoadNetwork&, bool>(),
           pybind11::arg("graph"),
           pybind11::arg("useCache"))
      .def(pybind11::init<dsf::RoadNetwork&, bool, std::optional<unsigned int>>(),
           pybind11::arg("graph"),
           pybind11::arg("useCache"),
           pybind11::arg("seed"))
      .def(pybind11::init<dsf::RoadNetwork&, bool, std::optional<unsigned int>, double>(),
           pybind11::arg("graph"),
           pybind11::arg("useCache"),
           pybind11::arg("seed"),
           pybind11::arg("alpha"))
      .def(pybind11::init<
               dsf::RoadNetwork&,
               bool,
               std::optional<unsigned int>,
               double,
               std::function<double(const dsf::RoadNetwork*, dsf::Id, dsf::Id)>>(),
           pybind11::arg("graph"),
           pybind11::arg("useCache"),
           pybind11::arg("seed"),
           pybind11::arg("alpha"),
           pybind11::arg("weightFunction"))
      .def(
          pybind11::init<dsf::RoadNetwork&,
                         bool,
                         std::optional<unsigned int>,
                         double,
                         std::function<double(const dsf::RoadNetwork*, dsf::Id, dsf::Id)>,
                         double>(),
          pybind11::arg("graph"),
          pybind11::arg("useCache"),
          pybind11::arg("seed"),
          pybind11::arg("alpha"),
          pybind11::arg("weightFunction"),
          pybind11::arg("weightTreshold"))
      .def("setForcePriorities",
           &dsf::FirstOrderDynamics::setForcePriorities,
           pybind11::arg("forcePriorities"))
      .def("setDataUpdatePeriod",
           &dsf::FirstOrderDynamics::setDataUpdatePeriod,
           pybind11::arg("dataUpdatePeriod"))
      .def("setMaxDistance",
           &dsf::FirstOrderDynamics::setMaxDistance,
           pybind11::arg("maxDistance"))
      .def("setMaxTravelTime",
           &dsf::FirstOrderDynamics::setMaxTravelTime,
           pybind11::arg("maxTravelTime"))
      .def("setDestinationNodes",
           static_cast<void (dsf::FirstOrderDynamics::*)(std::initializer_list<dsf::Id>,
                                                         bool)>(
               &dsf::FirstOrderDynamics::setDestinationNodes),
           pybind11::arg("destinationNodes"),
           pybind11::arg("updatePaths") = true)
      .def("updatePaths", &dsf::FirstOrderDynamics::updatePaths)
      .def("addAgentsUniformly",
           &dsf::FirstOrderDynamics::addAgentsUniformly,
           pybind11::arg("nAgents"),
           pybind11::arg("itineraryId") = std::nullopt)
      .def(
          "addAgentsRandomly",
          [](dsf::FirstOrderDynamics& self,
             dsf::Size nAgents,
             const std::variant<std::monostate, size_t, double>& minNodeDistance) {
            self.addAgentsRandomly(nAgents, minNodeDistance);
          },
          pybind11::arg("nAgents"),
          pybind11::arg("minNodeDistance") = std::monostate{})
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
          pybind11::arg("minNodeDistance"))
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
          pybind11::arg("minNodeDistance"))
      // .def("addAgent", static_cast<void (dsf::FirstOrderDynamics::*)(std::unique_ptr<dsf::Agent>)>(&dsf::FirstOrderDynamics::addAgent), pybind11::arg("agent"))
      // .def("addItinerary", static_cast<void (dsf::FirstOrderDynamics::*)(dsf::Id, dsf::Id)>(&dsf::FirstOrderDynamics::addItinerary), pybind11::arg("id"), pybind11::arg("destination"))
      // .def("addItinerary", static_cast<void (dsf::FirstOrderDynamics::*)(std::unique_ptr<dsf::Itinerary>)>(&dsf::FirstOrderDynamics::addItinerary), pybind11::arg("itinerary"))
      .def("evolve",
           &dsf::FirstOrderDynamics::evolve,
           pybind11::arg("reinsert_agents") = false)
      // .def("optimizeTrafficLights", &dsf::FirstOrderDynamics::optimizeTrafficLights,
      //      pybind11::arg("optimizationType") = dsf::TrafficLightOptimization::DOUBLE_TAIL,
      //      pybind11::arg("logFile") = "",
      //      pybind11::arg("threshold") = 0.,
      //      pybind11::arg("ratio") = 1.3)
      // .def("itineraries", &dsf::FirstOrderDynamics::itineraries, pybind11::return_value_policy::reference_internal)
      // .def("transitionMatrix", &dsf::FirstOrderDynamics::transitionMatrix, pybind11::return_value_policy::reference_internal)
      // .def("agents", &dsf::FirstOrderDynamics::agents, pybind11::return_value_policy::reference_internal)
      .def("nAgents", &dsf::FirstOrderDynamics::nAgents)
      .def("meanTravelTime",
           &dsf::FirstOrderDynamics::meanTravelTime,
           pybind11::arg("clearData") = false)
      .def("meanTravelDistance",
           &dsf::FirstOrderDynamics::meanTravelDistance,
           pybind11::arg("clearData") = false)
      .def("meanTravelSpeed",
           &dsf::FirstOrderDynamics::meanTravelSpeed,
           pybind11::arg("clearData") = false)
      .def("turnCounts",
           &dsf::FirstOrderDynamics::turnCounts,
           pybind11::return_value_policy::reference_internal)
      .def("turnProbabilities",
           &dsf::FirstOrderDynamics::turnProbabilities,
           pybind11::arg("reset") = true)
      .def("turnMapping",
           &dsf::FirstOrderDynamics::turnMapping,
           pybind11::return_value_policy::reference_internal)
      .def("agentMeanSpeed", &dsf::FirstOrderDynamics::agentMeanSpeed)
      // .def("streetMeanSpeed", static_cast<double (dsf::FirstOrderDynamics::*)(dsf::Id) const>(&dsf::FirstOrderDynamics::streetMeanSpeed), pybind11::arg("streetId"))
      // .def("streetMeanSpeed", static_cast<dsf::Measurement<double> (dsf::FirstOrderDynamics::*)() const>(&dsf::FirstOrderDynamics::streetMeanSpeed))
      // .def("streetMeanSpeed", static_cast<dsf::Measurement<double> (dsf::FirstOrderDynamics::*)(double, bool) const>(&dsf::FirstOrderDynamics::streetMeanSpeed), pybind11::arg("threshold"), pybind11::arg("above"))
      // .def("streetMeanDensity", &dsf::FirstOrderDynamics::streetMeanDensity, pybind11::arg("normalized") = false)
      // .def("streetMeanFlow", static_cast<dsf::Measurement<double> (dsf::FirstOrderDynamics::*)() const>(&dsf::FirstOrderDynamics::streetMeanFlow))
      // .def("streetMeanFlow", static_cast<dsf::Measurement<double> (dsf::FirstOrderDynamics::*)(double, bool) const>(&dsf::FirstOrderDynamics::streetMeanFlow), pybind11::arg("threshold"), pybind11::arg("above"))
      .def("meanSpireInputFlow",
           &dsf::FirstOrderDynamics::meanSpireInputFlow,
           pybind11::arg("resetValue") = true)
      .def("meanSpireOutputFlow",
           &dsf::FirstOrderDynamics::meanSpireOutputFlow,
           pybind11::arg("resetValue") = true)
      .def("saveStreetDensities",
           &dsf::FirstOrderDynamics::saveStreetDensities,
           pybind11::arg("filename"),
           pybind11::arg("normalized") = true,
           pybind11::arg("separator") = ';')
      .def("saveInputStreetCounts",
           &dsf::FirstOrderDynamics::saveInputStreetCounts,
           pybind11::arg("filename"),
           pybind11::arg("reset") = false,
           pybind11::arg("separator") = ';')
      .def("saveOutputStreetCounts",
           &dsf::FirstOrderDynamics::saveOutputStreetCounts,
           pybind11::arg("filename"),
           pybind11::arg("reset") = false,
           pybind11::arg("separator") = ';')
      .def("saveTravelSpeeds",
           &dsf::FirstOrderDynamics::saveTravelSpeeds,
           pybind11::arg("filename"),
           pybind11::arg("reset") = false)
      .def("saveMacroscopicObservables",
           &dsf::FirstOrderDynamics::saveMacroscopicObservables,
           pybind11::arg("filename"),
           pybind11::arg("separator") = ';');
}