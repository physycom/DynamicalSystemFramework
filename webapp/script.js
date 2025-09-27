// Initialize the Leaflet map
const baseZoom = 13;
const map = L.map('map').setView([0, 0], 1);

// Add OpenStreetMap tile layer with inverted grayscale effect
const tileLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OSM</a>'
}).addTo(map);
tileLayer.getContainer().style.filter = 'grayscale(100%) invert(100%)';

// Create an overlay for D3 visualizations
L.svg().addTo(map);
const overlay = d3.select(map.getPanes().overlayPane).select("svg");
const g = overlay.append("g").attr("class", "leaflet-zoom-hide");

let edges, densities;
let timeStep = 0;
let highlightedEdge = null;
let highlightedNode = null;

function formatTime(seconds) {
  const hours = Math.floor(seconds / 3600);
  const minutes = Math.floor((seconds % 3600) / 60);
  return `${hours.toString().padStart(2, '0')}:${minutes.toString().padStart(2, '0')}`;
}

// Create a color scale for density values using three color stops
const colorScale = d3.scaleLinear()
  .domain([0, 0.5, 1])
  .range(["green", "yellow", "red"]);

// Data folder selector
const dataFolderInput = document.getElementById('dataFolder');
dataFolderInput.addEventListener('change', function(event) {
  const files = Array.from(event.target.files);
  const edgesFile = files.find(f => f.name === 'edges.csv');
  const densitiesFile = files.find(f => f.name === 'densities.csv');

  if (!edgesFile || !densitiesFile) {
    alert('Please select a folder containing edges.csv and densities.csv');
    return;
  }

  // Create object URLs for the files
  const edgesUrl = URL.createObjectURL(edgesFile);
  const densitiesUrl = URL.createObjectURL(densitiesFile);

  // Load CSV data
  Promise.all([
    d3.dsv(";", edgesUrl, parseEdges),
    d3.dsv(";", densitiesUrl, parseDensity)
  ]).then(([edgesData, densityData]) => {
    edges = edgesData;
    densities = densityData;

        // console.log("Edges:", edges);
        // console.log("Densities:", densities);

    if (!edges.length || !densities.length) {
      console.error("Missing CSV data.");
      return;
    }

    // Calculate median center from edge geometries
    let allLats = [];
    let allLons = [];
    edges.forEach(edge => {
      if (edge.geometry && edge.geometry.length > 0) {
        edge.geometry.forEach(pt => {
          allLats.push(pt.y);
          allLons.push(pt.x);
        });
      }
    });
    if (allLats.length > 0 && allLons.length > 0) {
      allLats.sort((a, b) => a - b);
      allLons.sort((a, b) => a - b);
      const medianLat = allLats[Math.floor(allLats.length / 2)];
      const medianLon = allLons[Math.floor(allLons.length / 2)];
      map.setView([medianLat, medianLon], baseZoom);
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
        alert(`Edge ID: ${d.id} - from ${d.source} to ${d.target}.\n\nDensity at time step ${timeStep}:  ${densityValue}`);
      });

    // Function to update edge positions, and color edges based on density
    function update() {
      // Update edge paths
      link.attr("d", d => {
        if (d.geometry && d.geometry.length > 0) {
          const projectedCoords = d.geometry.map(pt => {
            const point = map.latLngToLayerPoint([pt.y, pt.x]);
            return [point.x, point.y];
          });
          return lineGenerator(projectedCoords);
        }
      });
      // Update edge stroke width based on zoom level
      function updateEdgeStrokeWidth() {
        const zoomLevel = map.getZoom();
        const strokeWidthScale = 3 + (zoomLevel - baseZoom);
        link.attr("stroke-width", strokeWidthScale);
      }

      // Add event listener to map
      map.on('zoomend', function() {
        updateEdgeStrokeWidth();
      });

      // Initial render (default zoom level)
      updateEdgeStrokeWidth();

      updateDensityVisualization();
      updateNodeHighlight();
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
        const rgb = d3.rgb(colorScale(density));
        let color = `rgba(${rgb.r}, ${rgb.g}, ${rgb.b}, 0.8)`;
        if (highlightedEdge && edge.id == highlightedEdge) {
          color = "white";
        }
        link.filter((d, i) => i === index)
            .attr("stroke", color);
      });
    }

    // Update node highlight position
    function updateNodeHighlight() {
      g.selectAll(".node-highlight").remove();
      if (highlightedNode) {
        const point = map.latLngToLayerPoint([highlightedNode.y, highlightedNode.x]);
        g.append("circle")
          .attr("class", "node-highlight")
          .attr("cx", point.x)
          .attr("cy", point.y)
          .attr("r", 10)
          .attr("fill", "white")
          .attr("stroke", "white")
          .attr("stroke-width", 2);
      }
    }

    // Set up the time slider based on the density data's maximum time value
    const maxTimeStep = d3.max(densities, d => d.time);
    const timeSlider = document.getElementById('timeSlider');
    const timeLabel = document.getElementById('timeLabel');
    // Round up max to the nearest 300 for step consistency
    timeSlider.max = Math.ceil(maxTimeStep / 300) * 300;
    timeSlider.step = 300;
    timeLabel.textContent = `Time Step: ${formatTime(timeStep)}`;

    // Update the visualization when the slider value changes
    timeSlider.addEventListener('input', function() {
      timeStep = parseInt(timeSlider.value);
      timeLabel.textContent = `Time Step: ${formatTime(timeStep)}`;
      update();
    });

    // Edge search
    const edgeSearchBtn = document.getElementById('edgeSearchBtn');
    edgeSearchBtn.addEventListener('click', () => {
      const id = document.getElementById('edgeSearch').value.trim();
      const edge = edges.find(e => e.id == id);
      if (edge) {
        highlightedEdge = id;
        updateDensityVisualization();
        // Zoom to the edge
        if (edge.geometry && edge.geometry.length > 0) {
          const lats = edge.geometry.map(p => p.y);
          const lngs = edge.geometry.map(p => p.x);
          const minLat = Math.min(...lats);
          const maxLat = Math.max(...lats);
          const minLng = Math.min(...lngs);
          const maxLng = Math.max(...lngs);
          const bounds = L.latLngBounds([minLat, minLng], [maxLat, maxLng]);
          map.fitBounds(bounds, {padding: [20, 20]});
        }
        document.getElementById('searchResults').innerHTML = `
          <strong>Edge ID:</strong> ${edge.id}<br>
          <strong>Source:</strong> ${edge.source}<br>
          <strong>Target:</strong> ${edge.target}<br>
          <strong>Name:</strong> ${edge.name}
        `;
      } else {
        document.getElementById('searchResults').innerHTML = 'Edge not found';
      }
    });

    // Node search
    const nodeSearchBtn = document.getElementById('nodeSearchBtn');
    nodeSearchBtn.addEventListener('click', () => {
      const id = document.getElementById('nodeSearch').value.trim();
      const edgeAsSource = edges.find(e => e.source == id);
      const edgeAsTarget = edges.find(e => e.target == id);
      if (edgeAsSource) {
        const geom = edgeAsSource.geometry;
        if (geom && geom.length > 0) {
          highlightedNode = geom[0];
          updateNodeHighlight();
          map.setView([highlightedNode.y, highlightedNode.x], 18);
          document.getElementById('searchResults').innerHTML = `
            <strong>Node ID:</strong> ${id}<br>
            <strong>Position:</strong> (${highlightedNode.x}, ${highlightedNode.y})
          `;
        }
      } else if (edgeAsTarget) {
        const geom = edgeAsTarget.geometry;
        if (geom && geom.length > 0) {
          highlightedNode = geom[geom.length - 1];
          updateNodeHighlight();
          map.setView([highlightedNode.y, highlightedNode.x], 18);
          document.getElementById('searchResults').innerHTML = `
            <strong>Node ID:</strong> ${id}<br>
            <strong>Position:</strong> (${highlightedNode.x}, ${highlightedNode.y})
          `;
        }
      } else {
        document.getElementById('searchResults').innerHTML = 'Node not found';
      }
    });

    // Clear selections
    const clearBtn = document.getElementById('clearBtn');
    clearBtn.addEventListener('click', () => {
      highlightedEdge = null;
      highlightedNode = null;
      updateDensityVisualization();
      updateNodeHighlight();
      document.getElementById('searchResults').innerHTML = '';
      document.getElementById('edgeSearch').value = '';
      document.getElementById('nodeSearch').value = '';
    });

    // Hide data selector and show slider and search
    document.querySelector('.data-selector').style.display = 'none';
    document.querySelector('.slider-container').style.display = 'block';
    document.querySelector('.search-container').style.display = 'block';
  }).catch(error => {
    console.error("Error loading CSV files:", error);
  });
});

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
        id: d.id,
        source: d.source,
        target: d.target,
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