// Initialize the Leaflet map
const baseZoom = 13;
const map = L.map('map').setView([0, 0], 1);

// Add OpenStreetMap tile layer with inverted grayscale effect
const tileLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OSM</a>'
}).addTo(map);
tileLayer.getContainer().style.filter = 'grayscale(100%) invert(100%)';

// Add scale control
L.control.scale({
  position: 'bottomright',
  metric: true,
  imperial: false
}).addTo(map);

// Custom Canvas layer for edges
L.CanvasEdges = L.Layer.extend({
  initialize: function(edges, options) {
    L.setOptions(this, options);
    this.edges = edges;
    this.colors = [];
  },

  onAdd: function(map) {
    this._map = map;
    this._canvas = L.DomUtil.create('canvas', 'leaflet-canvas-layer');
    this._ctx = this._canvas.getContext('2d');
    
    // Set canvas style
    this._canvas.style.pointerEvents = 'none';
    this._canvas.style.position = 'absolute';
    this._canvas.style.top = '0';
    this._canvas.style.left = '0';
    
    map.getPanes().overlayPane.appendChild(this._canvas);
    
    // Bind events
    map.on('viewreset', this._reset, this);
    map.on('zoom', this._update, this);
    map.on('zoomstart', this._onZoomStart, this);
    map.on('zoomend', this._onZoomEnd, this);
    map.on('move', this._update, this);
    map.on('moveend', this._update, this);
    map.on('click', this._onMapClick, this);
    map.on('mousemove', this._onMouseMove, this);
    
    this._reset();
  },

  onRemove: function(map) {
    map.getPanes().overlayPane.removeChild(this._canvas);
    map.off('viewreset', this._reset, this);
    map.off('zoom', this._update, this);
    map.off('zoomstart', this._onZoomStart, this);
    map.off('zoomend', this._onZoomEnd, this);
    map.off('move', this._update, this);
    map.off('moveend', this._update, this);
    map.off('click', this._onMapClick, this);
    map.off('mousemove', this._onMouseMove, this);
    
    if (this._zoomAnimationFrame) {
      cancelAnimationFrame(this._zoomAnimationFrame);
    }
  },

  _reset: function() {
    const size = this._map.getSize();
    this._canvas.width = size.x;
    this._canvas.height = size.y;
    
    const topLeft = this._map.containerPointToLayerPoint([0, 0]);
    L.DomUtil.setPosition(this._canvas, topLeft);
    
    this._update();
  },

  _update: function() {
    if (!this._map) return;
    
    const topLeft = this._map.containerPointToLayerPoint([0, 0]);
    L.DomUtil.setPosition(this._canvas, topLeft);
    
    this._redraw();
  },

  _onZoomStart: function() {
    this._zooming = true;
    // Hide edges during zoom by clearing the canvas
    this._ctx.clearRect(0, 0, this._canvas.width, this._canvas.height);
  },

  _onZoomEnd: function() {
    this._zooming = false;
    if (this._zoomAnimationFrame) {
      cancelAnimationFrame(this._zoomAnimationFrame);
      this._zoomAnimationFrame = null;
    }
    // Show edges again after zoom ends
    this._update();
  },

  _animateZoomUpdate: function() {
    if (!this._zooming) return;
    
    // Don't redraw edges during zoom animation for better performance
    const topLeft = this._map.containerPointToLayerPoint([0, 0]);
    L.DomUtil.setPosition(this._canvas, topLeft);
    
    // Continue updating during zoom animation
    this._zoomAnimationFrame = requestAnimationFrame(() => {
      this._animateZoomUpdate();
    });
  },

  setColors: function(colors) {
    this.colors = colors;
    this._redraw();
  },

  setHighlightedEdge: function(highlightedEdge) {
    this.highlightedEdge = highlightedEdge;
    this._redraw();
  },

  _redraw: function() {
    if (!this._map) return;
    
    // Don't draw edges while zooming for better performance
    if (this._zooming) return;
    
    const ctx = this._ctx;
    ctx.clearRect(0, 0, this._canvas.width, this._canvas.height);
    const zoom = this._map.getZoom();
    const strokeWidth = 3 + (zoom - baseZoom);

    this.edges.forEach((edge, index) => {
      if (!edge.geometry || edge.geometry.length === 0) return;

      ctx.beginPath();
      const firstPoint = this._map.latLngToContainerPoint([edge.geometry[0].y, edge.geometry[0].x]);
      ctx.moveTo(firstPoint.x, firstPoint.y);

      for (let i = 1; i < edge.geometry.length; i++) {
        const point = this._map.latLngToContainerPoint([edge.geometry[i].y, edge.geometry[i].x]);
        ctx.lineTo(point.x, point.y);
      }

      ctx.lineWidth = strokeWidth;
      ctx.lineCap = 'round';
      ctx.lineJoin = 'round';

      let color = this.colors[index] || 'rgba(255, 255, 255, 0.8)';
      if (this.highlightedEdge && edge.id == this.highlightedEdge) {
        color = 'white';
      }

      ctx.strokeStyle = color;
      ctx.stroke();

      // Draw dashed line for autostrada
      if (edge.name.toLowerCase().includes("autostrada")) {
        ctx.setLineDash([4, 4]);
        ctx.stroke();
        ctx.setLineDash([]);
      }
    });
  },

  _onMapClick: function(e) {
    const containerPoint = this._map.latLngToContainerPoint(e.latlng);
    const x = containerPoint.x;
    const y = containerPoint.y;

    let closestEdge = null;
    let minDist = Infinity;

    this.edges.forEach(edge => {
      if (!edge.geometry || edge.geometry.length < 2) return;

      for (let i = 0; i < edge.geometry.length - 1; i++) {
        const p1 = this._map.latLngToContainerPoint([edge.geometry[i].y, edge.geometry[i].x]);
        const p2 = this._map.latLngToContainerPoint([edge.geometry[i+1].y, edge.geometry[i+1].x]);
        const dist = this._pointToLineDistancePixels({x, y}, p1, p2);
        if (dist < minDist) {
          minDist = dist;
          closestEdge = edge;
        }
      }
    });

    if (closestEdge && minDist < 10) { // 10 pixel threshold
      highlightedEdge = closestEdge.id;
      highlightedNode = null;
      this.setHighlightedEdge(highlightedEdge);
      updateNodeHighlight();

      // Zoom to the edge
      if (closestEdge.geometry && closestEdge.geometry.length > 0) {
        const lats = closestEdge.geometry.map(p => p.y);
        const lngs = closestEdge.geometry.map(p => p.x);
        const minLat = Math.min(...lats);
        const maxLat = Math.max(...lats);
        const minLng = Math.min(...lngs);
        const maxLng = Math.max(...lngs);
        const bounds = L.latLngBounds([minLat, minLng], [maxLat, maxLng]);
        map.fitBounds(bounds, {padding: [20, 20]});
      }

      updateEdgeInfo(closestEdge);
      document.getElementById('inverseBtn').disabled = false;
    }
  },

  _pointToLineDistancePixels: function(point, lineStart, lineEnd) {
    const A = point.x - lineStart.x;
    const B = point.y - lineStart.y;
    const C = lineEnd.x - lineStart.x;
    const D = lineEnd.y - lineStart.y;

    const dot = A * C + B * D;
    const lenSq = C * C + D * D;
    let param = -1;
    if (lenSq !== 0) param = dot / lenSq;

    let xx, yy;
    if (param < 0) {
      xx = lineStart.x;
      yy = lineStart.y;
    } else if (param > 1) {
      xx = lineEnd.x;
      yy = lineEnd.y;
    } else {
      xx = lineStart.x + param * C;
      yy = lineStart.y + param * D;
    }

    const dx = point.x - xx;
    const dy = point.y - yy;
    return Math.sqrt(dx * dx + dy * dy);
  },

  _onMouseMove: function(e) {
    const containerPoint = this._map.latLngToContainerPoint(e.latlng);
    const x = containerPoint.x;
    const y = containerPoint.y;

    let minDist = Infinity;

    this.edges.forEach(edge => {
      if (!edge.geometry || edge.geometry.length < 2) return;

      for (let i = 0; i < edge.geometry.length - 1; i++) {
        const p1 = this._map.latLngToContainerPoint([edge.geometry[i].y, edge.geometry[i].x]);
        const p2 = this._map.latLngToContainerPoint([edge.geometry[i+1].y, edge.geometry[i+1].x]);
        const dist = this._pointToLineDistancePixels({x, y}, p1, p2);
        if (dist < minDist) {
          minDist = dist;
        }
      }
    });

    if (minDist < 10) { // Same threshold as click
      this._map.getContainer().style.cursor = 'pointer';
    } else {
      this._map.getContainer().style.cursor = '';
    }
  }
});

// Create an overlay for D3 visualizations (keeping for node highlights)
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

// Update edge info display with current density
function updateEdgeInfo(edge) {
  const edgeIndex = edges.indexOf(edge);
  const currentDensityRow = densities.find(d => d.time === timeStep);
  let density = 'N/A';
  if (currentDensityRow) {
    density = currentDensityRow.densities[edgeIndex];
    if (density === undefined || isNaN(density)) density = 0;
    density = parseFloat(density).toFixed(2);
  }
  document.getElementById('searchResults').innerHTML = `
    <strong>Edge ID:</strong> ${edge.id}<br>
    <strong>Source:</strong> ${edge.source}<br>
    <strong>Target:</strong> ${edge.target}<br>
    <strong>Name:</strong> ${edge.name}<br>
    <strong>Density:</strong> ${density}
  `;
}

// Data directory loader
const loadDataBtn = document.getElementById('loadDataBtn');
const dataDirInput = document.getElementById('dataDir');

dataDirInput.addEventListener('keydown', function(event) {
  if (event.key === 'Enter') {
    loadDataBtn.click();
  }
});

loadDataBtn.addEventListener('click', async function() {
  const input = document.getElementById('dataDir');
  const dirName = input.value.trim();
  
  if (!dirName) {
    alert('Please enter a data directory name');
    return;
  }

  try {
    // Show loading state
    loadDataBtn.textContent = 'Loading...';
    loadDataBtn.disabled = true;
    
    // Fetch CSV files from the data subdirectory
    const edgesUrl = `${dirName}/edges.csv`;
    const densitiesUrl = `${dirName}/densities.csv`;

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

    // Create Canvas layer for edges
    const canvasEdges = new L.CanvasEdges(edges);
    canvasEdges.addTo(map);

    // Function to update edge positions, and color edges based on density
    function update() {
      // Update edge stroke width based on zoom level (handled in Canvas layer)
      // No need to update paths, Canvas layer handles it

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

      const colors = edges.map((edge, index) => {
        let density = currentDensities[index];
        if (density === undefined || isNaN(density)) {
          density = 0;
        }
        const rgb = d3.rgb(colorScale(density));
        return `rgba(${rgb.r}, ${rgb.g}, ${rgb.b}, 0.69)`;
      });

      canvasEdges.setColors(colors);
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
      // Update edge info if an edge is selected
      if (highlightedEdge) {
        const edge = edges.find(e => e.id == highlightedEdge);
        if (edge) updateEdgeInfo(edge);
      }
    });

    // Edge search
    const edgeSearchBtn = document.getElementById('edgeSearchBtn');
    edgeSearchBtn.addEventListener('click', () => {
      const id = document.getElementById('edgeSearch').value.trim();
      const edge = edges.find(e => e.id == id);
      if (edge) {
        highlightedEdge = id;
        canvasEdges.setHighlightedEdge(highlightedEdge);
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
        updateEdgeInfo(edge);
        document.getElementById('inverseBtn').disabled = false;
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
      document.getElementById('inverseBtn').disabled = true;
    });

    // Clear selections
    const clearBtn = document.getElementById('clearBtn');
    clearBtn.addEventListener('click', () => {
      highlightedEdge = null;
      highlightedNode = null;
      canvasEdges.setHighlightedEdge(null);
      updateNodeHighlight();
      document.getElementById('searchResults').innerHTML = '';
      document.getElementById('edgeSearch').value = '';
      document.getElementById('nodeSearch').value = '';
      document.getElementById('inverseBtn').disabled = true;
    });

    // Inverse edge button
    const inverseBtn = document.getElementById('inverseBtn');
    inverseBtn.addEventListener('click', () => {
      if (!highlightedEdge) return;
      const currentEdge = edges.find(e => e.id == highlightedEdge);
      if (!currentEdge) return;
      
      // Find inverse edge: source == current target, target == current source
      const inverseEdge = edges.find(e => e.source == currentEdge.target && e.target == currentEdge.source);
      if (inverseEdge) {
        highlightedEdge = inverseEdge.id;
        highlightedNode = null;
        canvasEdges.setHighlightedEdge(highlightedEdge);
        updateNodeHighlight();
        // Zoom to the inverse edge
        if (inverseEdge.geometry && inverseEdge.geometry.length > 0) {
          const lats = inverseEdge.geometry.map(p => p.y);
          const lngs = inverseEdge.geometry.map(p => p.x);
          const minLat = Math.min(...lats);
          const maxLat = Math.max(...lats);
          const minLng = Math.min(...lngs);
          const maxLng = Math.max(...lngs);
          const bounds = L.latLngBounds([minLat, minLng], [maxLat, maxLng]);
          map.fitBounds(bounds, {padding: [20, 20]});
        }
        updateEdgeInfo(inverseEdge);
      } else {
        alert('Inverse edge from ' + currentEdge.target + ' to ' + currentEdge.source + ' not found');
      }
    });

      // Hide data selector and show slider and search
      document.querySelector('.data-selector').style.display = 'none';
      document.querySelector('.slider-container').style.display = 'block';
      document.querySelector('.search-container').style.display = 'block';
    }).catch(error => {
      console.error("Error loading CSV files:", error);
      alert('Error loading data files. Please check the console for details.');
    }).finally(() => {
      // Reset button state
      loadDataBtn.textContent = 'Load Data';
      loadDataBtn.disabled = false;
    });
  } catch (error) {
    console.error('Error:', error);
    alert('Error loading data. Please try again.');
    loadDataBtn.textContent = 'Load Data';
    loadDataBtn.disabled = false;
  }
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