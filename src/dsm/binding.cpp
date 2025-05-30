#include "./headers/AdjacencyMatrix.hpp"
#include "./headers/RoadNetwork.hpp"
#include "./headers/FirstOrderDynamics.hpp"

#include <pybind11/pybind11.h>
#include <pybind11/stl.h> // Changed to include all stl type casters

PYBIND11_MODULE(dsf, m) {
    m.doc() = "Python bindings for the DSM library";

    pybind11::class_<dsm::AdjacencyMatrix>(m, "AdjacencyMatrix")
        .def(pybind11::init<>())
        .def(pybind11::init<std::string const&>(), pybind11::arg("fileName")) // Added constructor
        .def("n", &dsm::AdjacencyMatrix::n)
        .def("size", &dsm::AdjacencyMatrix::size)
        .def("empty", &dsm::AdjacencyMatrix::empty) // Added empty
        .def("getRow", &dsm::AdjacencyMatrix::getRow)
        .def("getCol", &dsm::AdjacencyMatrix::getCol) // Added getCol
        .def("operator()", [](const dsm::AdjacencyMatrix& self, dsm::Id i, dsm::Id j) { return self(i, j); })
        .def("insert", &dsm::AdjacencyMatrix::insert) // Added insert
        .def("contains", &dsm::AdjacencyMatrix::contains) // Added contains
        .def("elements", &dsm::AdjacencyMatrix::elements) // Added elements
        .def("clear", &dsm::AdjacencyMatrix::clear)
        .def("clearRow", &dsm::AdjacencyMatrix::clearRow) // Added clearRow
        .def("clearCol", &dsm::AdjacencyMatrix::clearCol) // Added clearCol
        .def("getInDegreeVector", &dsm::AdjacencyMatrix::getInDegreeVector) // Added getInDegreeVector
        .def("getOutDegreeVector", &dsm::AdjacencyMatrix::getOutDegreeVector) // Added getOutDegreeVector
        .def("read", &dsm::AdjacencyMatrix::read, pybind11::arg("fileName")) // Added read
        .def("save", &dsm::AdjacencyMatrix::save, pybind11::arg("fileName")); // Added save

    pybind11::class_<dsm::RoadNetwork>(m, "RoadNetwork")
        .def(pybind11::init<>())
        .def(pybind11::init<const dsm::AdjacencyMatrix&>())
        .def("buildAdj", &dsm::RoadNetwork::buildAdj)
        .def("adjustNodeCapacities", &dsm::RoadNetwork::adjustNodeCapacities)
        .def("initTrafficLights", &dsm::RoadNetwork::initTrafficLights, pybind11::arg("minGreenTime") = 30)
        .def("autoMapStreetLanes", &dsm::RoadNetwork::autoMapStreetLanes)
        .def("importMatrix", &dsm::RoadNetwork::importMatrix, pybind11::arg("fileName"), pybind11::arg("isAdj") = true, pybind11::arg("defaultSpeed") = 13.8888888889)
        .def("importCoordinates", &dsm::RoadNetwork::importCoordinates, pybind11::arg("fileName"))
        .def("importOSMNodes", &dsm::RoadNetwork::importOSMNodes, pybind11::arg("fileName"))
        .def("importOSMEdges", &dsm::RoadNetwork::importOSMEdges, pybind11::arg("fileName"))
        .def("importTrafficLights", &dsm::RoadNetwork::importTrafficLights, pybind11::arg("fileName"))
        .def("exportNodes", &dsm::RoadNetwork::exportNodes, pybind11::arg("fileName"))
        .def("exportEdges", &dsm::RoadNetwork::exportEdges, pybind11::arg("fileName"));

    pybind11::class_<dsm::Itinerary>(m, "Itinerary")
        .def(pybind11::init<dsm::Id, dsm::Id>(), pybind11::arg("id"), pybind11::arg("destination"))
        .def("setPath", &dsm::Itinerary::setPath, pybind11::arg("path"))
        .def("id", &dsm::Itinerary::id)
        .def("destination", &dsm::Itinerary::destination);
        // .def("path", &dsm::Itinerary::path, pybind11::return_value_policy::reference_internal);

    pybind11::class_<dsm::FirstOrderDynamics>(m, "Dynamics")
    //     // Constructors are not directly exposed due to the template nature and complexity.
    //     // Users should use derived classes like FirstOrderDynamics.
        .def(pybind11::init<dsm::RoadNetwork&>(),
             pybind11::arg("graph"))
        .def(pybind11::init<dsm::RoadNetwork&, bool>(),
             pybind11::arg("graph"),
             pybind11::arg("useCache"))
        .def(pybind11::init<dsm::RoadNetwork&, bool, std::optional<unsigned int>>(),
             pybind11::arg("graph"),
             pybind11::arg("useCache"),
             pybind11::arg("seed"))
        .def(pybind11::init<dsm::RoadNetwork&, bool, std::optional<unsigned int>, double>(),
             pybind11::arg("graph"),
             pybind11::arg("useCache"),
             pybind11::arg("seed"),
             pybind11::arg("alpha"))
        .def(pybind11::init<dsm::RoadNetwork&, bool, std::optional<unsigned int>, double, std::function<double(const dsm::RoadNetwork*, dsm::Id, dsm::Id)>>(),
             pybind11::arg("graph"),
             pybind11::arg("useCache"),
             pybind11::arg("seed"),
             pybind11::arg("alpha"),
             pybind11::arg("weightFunction"))
        .def(pybind11::init<dsm::RoadNetwork&, bool, std::optional<unsigned int>, double, std::function<double(const dsm::RoadNetwork*, dsm::Id, dsm::Id)>, double>(),
             pybind11::arg("graph"),
             pybind11::arg("useCache"),
             pybind11::arg("seed"),
             pybind11::arg("alpha"),
             pybind11::arg("weightFunction"),
             pybind11::arg("weightTreshold"))
        .def("setForcePriorities", &dsm::FirstOrderDynamics::setForcePriorities, pybind11::arg("forcePriorities"))
        .def("setDataUpdatePeriod", &dsm::FirstOrderDynamics::setDataUpdatePeriod, pybind11::arg("dataUpdatePeriod"))
        .def("setMaxDistance", &dsm::FirstOrderDynamics::setMaxDistance, pybind11::arg("maxDistance"))
        .def("setMaxTravelTime", &dsm::FirstOrderDynamics::setMaxTravelTime, pybind11::arg("maxTravelTime"))
        .def("setDestinationNodes", static_cast<void (dsm::FirstOrderDynamics::*)(std::initializer_list<dsm::Id>, bool)>(&dsm::FirstOrderDynamics::setDestinationNodes), pybind11::arg("destinationNodes"), pybind11::arg("updatePaths") = true)
        .def("updatePaths", &dsm::FirstOrderDynamics::updatePaths)
        .def("addAgentsUniformly", &dsm::FirstOrderDynamics::addAgentsUniformly, pybind11::arg("nAgents"), pybind11::arg("itineraryId") = std::nullopt)
        .def("addAgentsRandomly", [](dsm::FirstOrderDynamics& self, dsm::Size nAgents, const std::unordered_map<dsm::Id, double>& src_weights, const std::unordered_map<dsm::Id, double>& dst_weights, const std::variant<std::monostate, size_t, double>& minNodeDistance) {
            self.addAgentsRandomly(nAgents, src_weights, dst_weights, minNodeDistance);
        }, pybind11::arg("nAgents"), pybind11::arg("src_weights"), pybind11::arg("dst_weights"), pybind11::arg("minNodeDistance"))
        .def("addAgentsRandomly", [](dsm::FirstOrderDynamics& self, dsm::Size nAgents, const std::map<dsm::Id, double>& src_weights, const std::map<dsm::Id, double>& dst_weights, const std::variant<std::monostate, size_t, double>& minNodeDistance) {
            self.addAgentsRandomly(nAgents, src_weights, dst_weights, minNodeDistance);
        }, pybind11::arg("nAgents"), pybind11::arg("src_weights"), pybind11::arg("dst_weights"), pybind11::arg("minNodeDistance"))
        // .def("addAgent", static_cast<void (dsm::FirstOrderDynamics::*)(std::unique_ptr<dsm::Agent>)>(&dsm::FirstOrderDynamics::addAgent), pybind11::arg("agent"))
        // .def("addItinerary", static_cast<void (dsm::FirstOrderDynamics::*)(dsm::Id, dsm::Id)>(&dsm::FirstOrderDynamics::addItinerary), pybind11::arg("id"), pybind11::arg("destination"))
        // .def("addItinerary", static_cast<void (dsm::FirstOrderDynamics::*)(std::unique_ptr<dsm::Itinerary>)>(&dsm::FirstOrderDynamics::addItinerary), pybind11::arg("itinerary"))
        .def("evolve", &dsm::FirstOrderDynamics::evolve, pybind11::arg("reinsert_agents") = false)
        // .def("optimizeTrafficLights", &dsm::FirstOrderDynamics::optimizeTrafficLights,
        //      pybind11::arg("optimizationType") = dsm::TrafficLightOptimization::DOUBLE_TAIL,
        //      pybind11::arg("logFile") = "",
        //      pybind11::arg("threshold") = 0.,
        //      pybind11::arg("ratio") = 1.3)
        // .def("itineraries", &dsm::FirstOrderDynamics::itineraries, pybind11::return_value_policy::reference_internal)
        // .def("transitionMatrix", &dsm::FirstOrderDynamics::transitionMatrix, pybind11::return_value_policy::reference_internal)
        // .def("agents", &dsm::FirstOrderDynamics::agents, pybind11::return_value_policy::reference_internal)
        .def("nAgents", &dsm::FirstOrderDynamics::nAgents)
        .def("meanTravelTime", &dsm::FirstOrderDynamics::meanTravelTime, pybind11::arg("clearData") = false)
        .def("meanTravelDistance", &dsm::FirstOrderDynamics::meanTravelDistance, pybind11::arg("clearData") = false)
        .def("meanTravelSpeed", &dsm::FirstOrderDynamics::meanTravelSpeed, pybind11::arg("clearData") = false)
        .def("turnCounts", &dsm::FirstOrderDynamics::turnCounts, pybind11::return_value_policy::reference_internal)
        .def("turnProbabilities", &dsm::FirstOrderDynamics::turnProbabilities, pybind11::arg("reset") = true)
        .def("turnMapping", &dsm::FirstOrderDynamics::turnMapping, pybind11::return_value_policy::reference_internal)
        .def("agentMeanSpeed", &dsm::FirstOrderDynamics::agentMeanSpeed)
        // .def("streetMeanSpeed", static_cast<double (dsm::FirstOrderDynamics::*)(dsm::Id) const>(&dsm::FirstOrderDynamics::streetMeanSpeed), pybind11::arg("streetId"))
        // .def("streetMeanSpeed", static_cast<dsm::Measurement<double> (dsm::FirstOrderDynamics::*)() const>(&dsm::FirstOrderDynamics::streetMeanSpeed))
        // .def("streetMeanSpeed", static_cast<dsm::Measurement<double> (dsm::FirstOrderDynamics::*)(double, bool) const>(&dsm::FirstOrderDynamics::streetMeanSpeed), pybind11::arg("threshold"), pybind11::arg("above"))
        // .def("streetMeanDensity", &dsm::FirstOrderDynamics::streetMeanDensity, pybind11::arg("normalized") = false)
        // .def("streetMeanFlow", static_cast<dsm::Measurement<double> (dsm::FirstOrderDynamics::*)() const>(&dsm::FirstOrderDynamics::streetMeanFlow))
        // .def("streetMeanFlow", static_cast<dsm::Measurement<double> (dsm::FirstOrderDynamics::*)(double, bool) const>(&dsm::FirstOrderDynamics::streetMeanFlow), pybind11::arg("threshold"), pybind11::arg("above"))
        .def("meanSpireInputFlow", &dsm::FirstOrderDynamics::meanSpireInputFlow, pybind11::arg("resetValue") = true)
        .def("meanSpireOutputFlow", &dsm::FirstOrderDynamics::meanSpireOutputFlow, pybind11::arg("resetValue") = true)
        .def("saveStreetDensities", &dsm::FirstOrderDynamics::saveStreetDensities, pybind11::arg("filename"), pybind11::arg("normalized") = true, pybind11::arg("separator") = ';')
        .def("saveInputStreetCounts", &dsm::FirstOrderDynamics::saveInputStreetCounts, pybind11::arg("filename"), pybind11::arg("reset") = false, pybind11::arg("separator") = ';')
        .def("saveOutputStreetCounts", &dsm::FirstOrderDynamics::saveOutputStreetCounts, pybind11::arg("filename"), pybind11::arg("reset") = false, pybind11::arg("separator") = ';')
        .def("saveTravelSpeeds", &dsm::FirstOrderDynamics::saveTravelSpeeds, pybind11::arg("filename"), pybind11::arg("reset") = false)
        .def("saveMacroscopicObservables", &dsm::FirstOrderDynamics::saveMacroscopicObservables, pybind11::arg("filename"), pybind11::arg("separator") = ';');
}