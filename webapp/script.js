// Initialize the Leaflet map
const baseZoom = 13;
const map = L.map('map').setView([44.4949, 11.3426], baseZoom); // Centered on Bologna, Italy

// Add OpenStreetMap tile layer with inverted grayscale effect
const tileLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OSM</a>'
}).addTo(map);
tileLayer.getContainer().style.filter = 'grayscale(100%) invert(100%)';

// Create an overlay for D3 visualizations
L.svg().addTo(map);
const overlay = d3.select(map.getPanes().overlayPane).select("svg");
const g = overlay.append("g").attr("class", "leaflet-zoom-hide");

let nodes, edges, densities;
let timeStep = 0;

// Load CSV data for nodes, edges, and densities
Promise.all([
  d3.dsv(";", "./data/nodes.csv", parseNodes),
  d3.dsv(";", "./data/edges.csv", parseEdges),
  d3.dsv(";", "./data/densities.csv", parseDensity)
]).then(([nodesData, edgesData, densityData]) => {
  nodes = nodesData;
  edges = edgesData;
  densities = densityData;

      // console.log("Nodes:", nodes);
      // console.log("Edges:", edges);
      // console.log("Densities:", densities);

  if (!nodes.length || !edges.length || !densities.length) {
    console.error("Missing CSV data.");
    return;
  }

  // Create a map of nodes keyed by their id for quick lookup
  const nodeMap = new Map(nodes.map(d => [d.id, d]));

  // Filter out edges whose nodes do not exist
  edges = edges.filter(d => nodeMap.has(d.u) && nodeMap.has(d.v));

  // Create a color scale for density values using three color stops
  const colorScale = d3.scaleLinear()
    .domain([0, 0.5, 1])
    .range(["green", "yellow", "red"]);

  // Function to project geographic coordinates into Leaflet's layer point coordinates
  function project(d) {
    return map.latLngToLayerPoint([d.y, d.x]);
  }

  // D3 line generator to draw paths
  const lineGenerator = d3.line()
    .x(d => d[0])
    .y(d => d[1]);

  // Draw edges as SVG paths
  const link = g.selectAll("path")
    .data(edges)
    .enter()
    .append("path")
    .attr("fill", "none")
    .attr("stroke", "white")
    .attr("stroke-dasharray", d => 
        d.name.toLowerCase().includes("autostrada") ? "4,4" : "none"
    )
    .style("pointer-events", "all")
    .style("cursor", "pointer")
    .on("click", function(event, d) {
      const densityData = densities.find(row => row.time === timeStep);
      const densityValue = densityData ? densityData.densities[edges.indexOf(d)] : "N/A";
      alert(`Edge ID: ${d.osm_id} - from ${d.u} to ${d.v}.\n\nDensity at time step ${timeStep}:  ${densityValue}`);
    });


  // Draw nodes as SVG circles
  const node = g.selectAll("circle")
    .data(nodes)
    .enter()
    .append("circle")
    .attr("fill", "blue")
    .style("cursor", "pointer")
    .on("click", function(event, d) {
      alert(`Node ID: ${d.id}`);
    });

  // Function to update node and edge positions, and color edges based on density
  function update() {
    // Project nodes to current map coordinates
    nodes.forEach(d => d.projected = project(d));

    // Update edge paths
    link.attr("d", d => {
      if (d.geometry && d.geometry.length > 0) {
        const projectedCoords = d.geometry.map(pt => {
          const point = map.latLngToLayerPoint([pt.y, pt.x]);
          return [point.x, point.y];
        });
        return lineGenerator(projectedCoords);
      } else {
        // Fallback: draw a straight line between the two nodes
        const start = project(nodeMap.get(d.u));
        const end = project(nodeMap.get(d.v));
        return lineGenerator([[start.x, start.y], [end.x, end.y]]);
      }
    });

    // Update node positions
    node.attr("cx", d => d.projected.x)
        .attr("cy", d => d.projected.y);


    // Update node radius based on zoom level
    function updateNodeRadius() {
      const zoomLevel = map.getZoom();
      const radiusScale = 3 + (zoomLevel - baseZoom);
      node.attr("r", radiusScale);
    }
    // Update edge stroke width based on zoom level
    function updateEdgeStrokeWidth() {
      const zoomLevel = map.getZoom();
      const strokeWidthScale = 3 + (zoomLevel - baseZoom);
      link.attr("stroke-width", strokeWidthScale);
    }

    // Add event listener to map
    map.on('zoomend', function() {
      updateNodeRadius();
      updateEdgeStrokeWidth();
    });

    // Initial render (default zoom level)
    updateNodeRadius();
    updateEdgeStrokeWidth();

    updateDensityVisualization();
  }

  map.on("zoomend", update);
  update(); // Initial render

  // Update edge colors based on the current time step density data
  function updateDensityVisualization() {
    const currentDensityRow = densities.find(d => d.time === timeStep);
    if (!currentDensityRow) {
      console.error("No density data for time step:", timeStep);
      return;
    }
    const currentDensities = currentDensityRow.densities;

    // For each edge, update the stroke color based on its density value
    edges.forEach((edge, index) => {
      let density = currentDensities[index];
      if (density === undefined || isNaN(density)) {
        console.warn(`Edge index ${index} has invalid density. Defaulting to 0.`);
        density = 0;
      }
      const color = colorScale(density);
      link.filter((d, i) => i === index)
          .attr("stroke", color);
    });
  }

  // Set up the time slider based on the density data's maximum time value
  const maxTimeStep = d3.max(densities, d => d.time);
  const timeSlider = document.getElementById('timeSlider');
  const timeLabel = document.getElementById('timeLabel');
  // Round up max to the nearest 300 for step consistency
  timeSlider.max = Math.ceil(maxTimeStep / 300) * 300;
  timeSlider.step = 300;
  timeLabel.textContent = `Time Step: ${timeStep}`;

  // Update the visualization when the slider value changes
  timeSlider.addEventListener('input', function() {
    timeStep = parseInt(timeSlider.value);
    timeLabel.textContent = `Time Step: ${timeStep}`;
    update();
  });
}).catch(error => {
  console.error("Error loading CSV files:", error);
});

// Parsing function for nodes CSV
function parseNodes(d) {
      return {
        id: d.id,
        x: +d.lon, // Longitude
        y: +d.lat  // Latitude
      };
}

    // Parsing function for edges CSV, including geometry parsing
function parseEdges(d) {
      let geometry = [];
      if (d.geometry) {
        const coordsStr = d.geometry.replace(/^LINESTRING\s*\(/, '').replace(/\)$/, '');
        geometry = coordsStr.split(",").map(coordStr => {
          const coords = coordStr.trim().split(/\s+/);
          return { x: +coords[0], y: +coords[1] };
        });
      }
      return {
        osm_id: d.id,
        u: d.source_id,
        v: d.target_id,
        name: d.name,
        geometry: geometry
      };
}

// Parsing function for density CSV
function parseDensity(d) {
      const time = +d.time;
      const densities = Object.keys(d)
        .filter(key => key !== 'time')
        .map(key => {
          const val = d[key] ? d[key].trim() : "";
          return val === "" ? 0 : +val;
        });
      return { time, densities };
}