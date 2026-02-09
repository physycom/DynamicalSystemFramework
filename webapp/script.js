// Initialize the Leaflet map
const baseZoom = 13;
const map = L.map('map').setView([0, 0], 1);

// Grufoony - 9/2/2026
// TODO: make this dynamic based on data range
const MAX_DENSITY = 200;
const MAX_DENSITY_INVERTED = 1 / MAX_DENSITY;

// Add OpenStreetMap tile layer with inverted grayscale effect
const tileLayer = L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  attribution: '&copy; <a href="https://www.openstreetmap.org/copyright">OSM</a>',
  crossOrigin: true
}).addTo(map);
tileLayer.getContainer().style.filter = 'grayscale(100%) invert(100%)';

// Add scale control
L.control.scale({
  position: 'bottomright',
  metric: true,
  imperial: false
}).addTo(map);

// Add background menu control
L.Control.BackgroundMenu = L.Control.extend({
  options: {
    position: 'topright'
  },

  onAdd: function(map) {
  const container = L.DomUtil.create('div', 'leaflet-control-settings');
  container.style.position = 'relative';

    const settingsBtn = L.DomUtil.create('button', 'leaflet-control-search-btn', container);
    settingsBtn.innerHTML = '⚙️';
    settingsBtn.title = 'Background Options';
    settingsBtn.onclick = () => {
      const settingsMenu = container.querySelector('.settings');
      if (settingsMenu.style.display === 'block') {
        settingsMenu.style.display = 'none';
      } else {
        settingsMenu.style.display = 'block';
        // Position settings menu to the left of the button
        settingsMenu.style.position = 'absolute';
        settingsMenu.style.top = `${settingsBtn.offsetTop}px`;
        settingsMenu.style.right = `${container.offsetWidth - settingsBtn.offsetLeft + settingsBtn.offsetWidth}px`;
        settingsMenu.style.width = '100px';
        settingsMenu.style.zIndex = '1000';
      }
    };

    const settingsMenu = L.DomUtil.create('div', 'settings', container);
    settingsMenu.style.display = 'none'; // hidden initially

    // Create buttons in menu
    const normalBtn = L.DomUtil.create('button', '', settingsMenu);
    normalBtn.innerHTML = 'Normal';
    normalBtn.onclick = () => {
      tileLayer.getContainer().style.filter = '';
      settingsMenu.style.display = 'none';
    };

    const grayBtn = L.DomUtil.create('button', '', settingsMenu);
    grayBtn.innerHTML = 'Grayscale';
    grayBtn.onclick = () => {
      tileLayer.getContainer().style.filter = 'grayscale(100%)';
      settingsMenu.style.display = 'none';
    };

    const invertedBtn = L.DomUtil.create('button', '', settingsMenu);
    invertedBtn.innerHTML = 'Inverted';
    invertedBtn.onclick = () => {
      tileLayer.getContainer().style.filter = 'grayscale(100%) invert(100%)';
      settingsMenu.style.display = 'none';
    };

    return container;
  }
});

// Add search menu control
L.Control.SearchMenu = L.Control.extend({
  options: {
    position: 'topright'
  },

  onAdd: function(map) {
    const container = L.DomUtil.create('div', 'leaflet-control-search');
    container.style.position = 'relative';

    const searchBtn = L.DomUtil.create('button', 'leaflet-control-search-btn', container);
    searchBtn.innerHTML = '🔍';
    searchBtn.title = 'Toggle Search Menu';
    searchBtn.onclick = () => {
      const searchContainer = document.querySelector('.search-container');
      if (searchContainer.style.display === 'block') {
        searchContainer.style.display = 'none';
      } else {
        searchContainer.style.display = 'block';
        // Position search menu to the left of the button
        searchContainer.style.position = 'absolute';
        searchContainer.style.top = `${searchBtn.offsetTop}px`;
        searchContainer.style.right = `${container.offsetWidth - searchBtn.offsetLeft + searchBtn.offsetWidth}px`;
        searchContainer.style.width = '200px';
        searchContainer.style.zIndex = '1000';
      }
    };

    return container;
  }
});

// Add background menu control to map
map.addControl(new L.Control.BackgroundMenu());

// Add search menu control to map
map.addControl(new L.Control.SearchMenu());

// Add chart toggle control
L.Control.ChartToggle = L.Control.extend({
  options: {
    position: 'topright'
  },

  onAdd: function(map) {
    const container = L.DomUtil.create('div', 'leaflet-control-chart-toggle');
    const button = L.DomUtil.create('button', 'leaflet-control-search-btn', container);
    
    button.innerHTML = '📈';
    button.title = 'Toggle Chart';
    button.onclick = () => {
      const chartContainer = document.querySelector('.chart-container');
      if (chartContainer.style.display === 'none' || chartContainer.style.display === '') {
        chartContainer.style.display = 'block';
        if (typeof chart !== 'undefined' && chart) {
          chart.resize();
        }
      } else {
        chartContainer.style.display = 'none';
      }
    };

    return container;
  }
});

// Add chart toggle control to map
map.addControl(new L.Control.ChartToggle());

// Add screenshot control
L.Control.Screenshot = L.Control.extend({
  options: {
    position: 'topleft'
  },

  onAdd: function(map) {
    const container = L.DomUtil.create('div', 'leaflet-control-screenshot');
    const button = L.DomUtil.create('a', 'leaflet-control-screenshot-button', container);
    
    button.innerHTML = '📷';
    button.href = '#';
    button.title = 'Take Screenshot';
    button.style.cssText = `
      width: 26px;
      height: 26px;
      line-height: 26px;
      display: block;
      text-align: center;
      text-decoration: none;
      color: black;
      background: white;
      border: 2px solid rgba(0,0,0,0.2);
      border-radius: 4px;
      box-shadow: 0 1px 5px rgba(0,0,0,0.4);
      font-size: 14px;
      margin-bottom: 5px;
    `;

    L.DomEvent.on(button, 'click', L.DomEvent.stopPropagation)
              .on(button, 'click', L.DomEvent.preventDefault)
              .on(button, 'click', this._takeScreenshot, this);

    return container;
  },

  _takeScreenshot: function() {
    // Show loading indicator
    const loadingDiv = document.createElement('div');
    loadingDiv.innerHTML = 'Generating screenshot...';
    loadingDiv.style.cssText = `
      position: fixed;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
      background: rgba(0,0,0,0.8);
      color: white;
      padding: 20px;
      border-radius: 10px;
      z-index: 10000;
      font-size: 16px;
    `;
    document.body.appendChild(loadingDiv);

    // Get current filter
    const currentFilter = tileLayer.getContainer().style.filter;

    // Capture the map
    html2canvas(document.getElementById('app-container'), {
      useCORS: true,
      allowTaint: false,
      scale: 2, // Higher resolution
      width: window.innerWidth,
      height: window.innerHeight,
      ignoreElements: (element) => {
        // Ignore the loading indicator if it's inside app-container (it shouldn't be, but just in case)
        // Also ignore data selector if it's hidden (html2canvas usually handles hidden, but let's be safe)
        return element.classList.contains('data-selector') && element.style.display === 'none';
      },
      onclone: (clonedDoc) => {
        // Apply filter to tiles manually since html2canvas doesn't support CSS filters
        if (currentFilter && currentFilter !== 'none' && currentFilter !== '') {
          const clonedTileImages = clonedDoc.querySelectorAll('.leaflet-tile-pane img');
          const originalTileImages = document.querySelectorAll('.leaflet-tile-pane img');
          
          clonedTileImages.forEach((img, index) => {
            const originalImg = originalTileImages[index];
            if (originalImg && originalImg.complete) {
              const canvas = clonedDoc.createElement('canvas');
              canvas.width = originalImg.naturalWidth || originalImg.width;
              canvas.height = originalImg.naturalHeight || originalImg.height;
              
              // Copy styles
              canvas.style.cssText = img.style.cssText;
              canvas.className = img.className;
              
              const ctx = canvas.getContext('2d');
              ctx.filter = currentFilter;
              ctx.drawImage(originalImg, 0, 0, canvas.width, canvas.height);
              
              if (img.parentNode) {
                img.parentNode.replaceChild(canvas, img);
              }
            }
          });
        }
      }
    }).then(canvas => {
      // Remove loading indicator
      document.body.removeChild(loadingDiv);

      // Use full canvas dimensions without cropping
      const cropX = 0, cropY = 0, cropWidth = canvas.width, cropHeight = canvas.height;

      // Create PDF with canvas dimensions
      const { jsPDF } = window.jspdf;
      const pdf = new jsPDF({
        orientation: cropWidth > cropHeight ? 'landscape' : 'portrait',
        unit: 'px',
        format: [cropWidth, cropHeight]
      });

      // Use the full canvas directly
      const imgData = canvas.toDataURL('image/png');
      pdf.addImage(imgData, 'PNG', 0, 0, cropWidth, cropHeight);

      // Save the PDF
      const filename = `road_network_${new Date().toISOString().slice(0,19).replace(/:/g, '-')}.pdf`;
      pdf.save(filename);
    }).catch(error => {
      document.body.removeChild(loadingDiv);
      console.error('Screenshot failed:', error);
      alert('Screenshot failed. Please try again.');
    });
  },

  _getScaleText: function() {
    // Get the current scale from Leaflet scale control
    const scaleElement = document.querySelector('.leaflet-control-scale-line');
    if (scaleElement) {
      return scaleElement.textContent;
    }
    return 'Scale information not available';
  }
});

// Add screenshot control to map
map.addControl(new L.Control.Screenshot());

// Add MP4 recording control
L.Control.MP4Recorder = L.Control.extend({
  options: {
    position: 'topleft'
  },

  onAdd: function(map) {
    const container = L.DomUtil.create('div', 'leaflet-control-mp4');
    const button = L.DomUtil.create('a', 'leaflet-control-mp4-button', container);
    
    button.innerHTML = '🎥';
    button.href = '#';
    button.title = 'Record MP4';
    button.style.cssText = `
      width: 26px;
      height: 26px;
      line-height: 26px;
      display: block;
      text-align: center;
      text-decoration: none;
      color: black;
      background: white;
      border: 2px solid rgba(0,0,0,0.2);
      border-radius: 4px;
      box-shadow: 0 1px 5px rgba(0,0,0,0.4);
      font-size: 14px;
      margin-bottom: 5px;
    `;

    L.DomEvent.on(button, 'click', L.DomEvent.stopPropagation)
              .on(button, 'click', L.DomEvent.preventDefault)
              .on(button, 'click', this._startRecording, this);

    this.button = button;
    return container;
  },

  _startRecording: async function() {
    if (!densities || densities.length === 0) {
      alert('Please load data first.');
      return;
    }

    // Pause playback if active
    const playBtn = document.getElementById('playBtn');
    if (playBtn && playBtn.textContent === '⏸') {
      playBtn.click();
    }

    const fpsInput = document.getElementById('fpsInput');
    const fps = parseFloat(fpsInput.value) || 10;

    if (!confirm(`Start recording MP4 from current time to end?\nFPS: ${fps}\nNote: This process may take a while.`)) {
      return;
    }

    let isRecording = true;

    // Show loading/progress indicator
    const loadingDiv = document.createElement('div');
    loadingDiv.id = 'mp4-progress';
    loadingDiv.style.cssText = `
      position: fixed;
      top: 50%;
      left: 50%;
      transform: translate(-50%, -50%);
      background: rgba(0,0,0,0.8);
      color: white;
      padding: 20px;
      border-radius: 10px;
      z-index: 10000;
      font-size: 16px;
      text-align: center;
    `;
    loadingDiv.innerHTML = 'Initializing MP4 recorder...<br>';

    // Add Stop button
    const stopBtn = document.createElement('button');
    stopBtn.textContent = 'Stop & Save';
    stopBtn.style.cssText = 'margin-top: 10px; padding: 5px 10px; cursor: pointer;';
    stopBtn.onclick = () => {
      isRecording = false;
      stopBtn.disabled = true;
      stopBtn.textContent = 'Stopping...';
    };
    loadingDiv.appendChild(stopBtn);

    document.body.appendChild(loadingDiv);

    try {
      // Load mp4-muxer dynamically
      const { Muxer, ArrayBufferTarget } = await import('https://unpkg.com/mp4-muxer@5.1.4/build/mp4-muxer.mjs');

      const width = map.getSize().x;
      const height = map.getSize().y;
      // Ensure even dimensions for H.264
      const w = width % 2 === 0 ? width : width - 1;
      const h = height % 2 === 0 ? height : height - 1;

      const muxer = new Muxer({
        target: new ArrayBufferTarget(),
        video: {
          codec: 'avc',
          width: w,
          height: h
        },
        fastStart: 'in-memory'
      });

      const videoEncoder = new VideoEncoder({
        output: (chunk, meta) => muxer.addVideoChunk(chunk, meta),
        error: e => {
          console.error(e);
          alert("Video encoding error: " + e.message);
          isRecording = false;
        }
      });

      videoEncoder.configure({
        codec: 'avc1.4d002a', // Main Profile, Level 4.2 (supports up to 1080p)
        width: w,
        height: h,
        bitrate: 5_000_000 // 5 Mbps for better quality
      });

      const timeSlider = document.getElementById('timeSlider');
      const startVal = parseInt(timeSlider.value);
      const maxVal = parseInt(timeSlider.max);
      const step = parseInt(timeSlider.step);

      let currentVal = startVal;
      let frameIndex = 0;

      // Capture loop
      const captureFrame = async () => {
        if (!isRecording || currentVal > maxVal) {
          loadingDiv.innerHTML = 'Finalizing MP4...';
          await videoEncoder.flush();
          muxer.finalize();
          
          const buffer = muxer.target.buffer;
          const blob = new Blob([buffer], { type: 'video/mp4' });
          const url = URL.createObjectURL(blob);
          
          const a = document.createElement('a');
          a.href = url;
          a.download = `simulation_${new Date().toISOString().slice(0,19).replace(/:/g, '-')}.mp4`;
          document.body.appendChild(a);
          a.click();
          document.body.removeChild(a);
          window.URL.revokeObjectURL(url);
          
          if (document.getElementById('mp4-progress')) {
            document.body.removeChild(loadingDiv);
          }
          return;
        }

        // Update slider and map
        timeSlider.value = currentVal;
        timeSlider.dispatchEvent(new Event('input'));

        // Update progress text
        const progress = Math.round(((currentVal - startVal) / (maxVal - startVal)) * 100);
        
        // Clear previous content but keep the stop button
        loadingDiv.innerHTML = '';
        loadingDiv.appendChild(document.createTextNode(`Capturing frames: ${progress}%`));
        loadingDiv.appendChild(document.createElement('br'));
        loadingDiv.appendChild(document.createTextNode(`Time: ${document.getElementById('timeLabel').textContent}`));
        loadingDiv.appendChild(document.createElement('br'));
        loadingDiv.appendChild(stopBtn);

        // Wait a bit for render
        await new Promise(resolve => setTimeout(resolve, 200));

        try {
          const canvas = await html2canvas(document.getElementById('app-container'), {
            useCORS: true,
            allowTaint: false,
            logging: false,
            scale: 1,
            width: w,
            height: h,
            ignoreElements: (element) => {
               return element.style.display === 'none';
            },
            onclone: (clonedDoc) => {
              const currentFilter = tileLayer.getContainer().style.filter;
              if (currentFilter && currentFilter !== 'none') {
                const originalImages = document.querySelectorAll('.leaflet-tile-pane img');
                const clonedImages = clonedDoc.querySelectorAll('.leaflet-tile-pane img');
                
                clonedImages.forEach((img, index) => {
                  const original = originalImages[index];
                  if (original && original.complete) {
                    try {
                      const canvas = document.createElement('canvas');
                      canvas.width = original.naturalWidth;
                      canvas.height = original.naturalHeight;
                      const ctx = canvas.getContext('2d');
                      ctx.filter = currentFilter;
                      ctx.drawImage(original, 0, 0);
                      img.src = canvas.toDataURL();
                      img.style.filter = 'none';
                    } catch (e) {
                      // console.warn('Failed to apply filter to tile', e);
                    }
                  }
                });
                const layers = clonedDoc.querySelectorAll('.leaflet-tile-pane .leaflet-layer');
                layers.forEach(layer => {
                  layer.style.filter = 'none';
                });
              }
            }
          });
          
          // Create a VideoFrame from the canvas
          const frame = new VideoFrame(canvas, {
            timestamp: frameIndex * 1000000 / fps,
            duration: 1000000 / fps
          });

          videoEncoder.encode(frame, { keyFrame: frameIndex % 30 === 0 });
          frame.close();
          
          currentVal += step;
          frameIndex++;
          // Schedule next frame
          setTimeout(captureFrame, 0);
        } catch (err) {
          console.error(err);
          alert('Error capturing frame: ' + err.message);
          if (document.getElementById('mp4-progress')) {
            document.body.removeChild(loadingDiv);
          }
        }
      };

      captureFrame();

    } catch (err) {
      console.error(err);
      alert('Error initializing MP4 recorder: ' + err.message);
      if (document.getElementById('mp4-progress')) {
        document.body.removeChild(loadingDiv);
      }
    }
  }
});

// Add MP4 recorder control to map
map.addControl(new L.Control.MP4Recorder());

// Custom Canvas layer for edges
L.CanvasEdges = L.Layer.extend({
  initialize: function(edges, options) {
    L.setOptions(this, options);
    this.edges = edges;
    this.colors = [];
    this.densities = [];
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

  setDensities: function(densities) {
    this.densities = densities;
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
    const baseStrokeWidth = 3 + (zoom - baseZoom);

    this.edges.forEach((edge, index) => {
      if (!edge.geometry || edge.geometry.length === 0) return;

      ctx.beginPath();
      const firstPoint = this._map.latLngToContainerPoint([edge.geometry[0].y, edge.geometry[0].x]);
      ctx.moveTo(firstPoint.x, firstPoint.y);

      for (let i = 1; i < edge.geometry.length; i++) {
        const point = this._map.latLngToContainerPoint([edge.geometry[i].y, edge.geometry[i].x]);
        ctx.lineTo(point.x, point.y);
      }

      // Calculate width based on density
      let density = this.densities[index] || 0;
      // Scale width: base width + density factor
      density *= MAX_DENSITY_INVERTED;
      // Assuming density is roughly 0-1, but can be higher.
      // Let's cap the max width increase to avoid huge lines.
      const densityFactor = Math.min(density, 2.0); 
      ctx.lineWidth = Math.max(1, baseStrokeWidth * (0.5 + densityFactor));
      
      ctx.lineCap = 'round';
      ctx.lineJoin = 'round';

      let color = this.colors[index] || 'rgba(0, 128, 0, 0.69)';
      if (this.highlightedEdge && edge.id === this.highlightedEdge) {
        color = 'white';
        ctx.lineWidth = ctx.lineWidth * 1.5; // Make highlighted edge thicker
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

let edges, densities, globalData;
let timeStamp = new Date();
let highlightedEdge = null;
let highlightedNode = null;
let chart;
let db = null;
let selectedSimulationId = null;

function formatTime(date) {
  const year = date.getFullYear();
  const month = (date.getMonth() + 1).toString().padStart(2, '0');
  const day = date.getDate().toString().padStart(2, '0');
  const hours = date.getHours().toString().padStart(2, '0');
  const minutes = date.getMinutes().toString().padStart(2, '0');
  return `${year}-${month}-${day} ${hours}:${minutes}`;
}

// Create a color scale for density values using three color stops
const colorScale = d3.scaleLinear()
  .domain([0, MAX_DENSITY / 2, MAX_DENSITY])
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
  const currentDensityRow = densities.find(d => d.datetime.getTime() === timeStamp.getTime());
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
    <strong>Max Speed:</strong> ${edge.maxspeed || 'N/A'}<br>
    <strong>Name:</strong> ${edge.name}<br>
    <strong>Number of Lanes:</strong> ${edge.nlanes || 'N/A'}<br>
    <strong>Density:</strong> ${density}<br>
    <strong>Coil Code:</strong> ${edge.coilcode || 'N/A'}<br>
  `;
}

// Parse geometry from LINESTRING format (for SQL database)
function parseGeometry(geometryStr) {
  if (!geometryStr) return [];
  const coordsStr = geometryStr.replace(/^LINESTRING\s*\(/, '').replace(/\)$/, '');
  return coordsStr.split(",").map(coordStr => {
    const coords = coordStr.trim().split(/\s+/);
    return { x: +coords[0], y: +coords[1] };
  });
}

// Load edges from SQLite database
function loadEdgesFromDB() {
  const result = db.exec("SELECT id, source, target, length, maxspeed, name, nlanes, geometry FROM edges");
  if (result.length === 0) return [];
  
  const columns = result[0].columns;
  const values = result[0].values;
  
  return values.map(row => {
    const edge = {};
    columns.forEach((col, i) => {
      edge[col] = row[i];
    });
    edge.geometry = parseGeometry(edge.geometry);
    edge.maxspeed = +edge.maxspeed || 0;
    edge.nlanes = +edge.nlanes || 1;
    edge.length = +edge.length || 0;
    return edge;
  });
}

// Load road_data from SQLite for selected simulation and transform to density format
function loadRoadDataFromDB() {
  // Get distinct timestamps for the selected simulation
  const timestampResult = db.exec(
    `SELECT DISTINCT datetime FROM road_data WHERE simulation_id = ${selectedSimulationId} ORDER BY datetime`
  );
  if (timestampResult.length === 0) return [];
  
  const timestamps = timestampResult[0].values.map(row => row[0]);
  
  // Get edge IDs in order
  const edgeIds = edges.map(e => e.id);
  
  // Build density data for each timestamp
  const densityData = [];
  
  for (const ts of timestamps) {
    // Get all road_data for this timestamp and simulation
    const dataResult = db.exec(
      `SELECT street_id, density FROM road_data WHERE simulation_id = ${selectedSimulationId} AND datetime = '${ts}'`
    );
    
    const densityMap = {};
    if (dataResult.length > 0) {
      dataResult[0].values.forEach(row => {
        densityMap[row[0]] = row[1];
      });
    }
    
    // Build densities array in same order as edges
    const densityArray = edgeIds.map(id => densityMap[id] || 0);
    
    densityData.push({
      datetime: new Date(ts),
      densities: densityArray
    });
  }
  
  return densityData;
}

// Load global data (aggregated statistics per timestamp)
function loadGlobalDataFromDB() {
  // Calculate mean density, avg_speed, etc. per timestamp for selected simulation
  const result = db.exec(`
    SELECT datetime, 
           AVG(density) as mean_density,
           AVG(avg_speed) as mean_speed,
           SUM(counts) as total_counts
    FROM road_data 
    WHERE simulation_id = ${selectedSimulationId}
    GROUP BY datetime 
    ORDER BY datetime
  `);
  
  if (result.length === 0) return [];
  
  const columns = result[0].columns;
  const values = result[0].values;
  
  return values.map(row => {
    const data = { datetime: new Date(row[0]) };
    for (let i = 1; i < columns.length; i++) {
      data[columns[i]] = +row[i] || 0;
    }
    return data;
  });
}

// Get available simulations from database
function getSimulations() {
  const result = db.exec("SELECT id, name FROM simulations ORDER BY id");
  if (result.length === 0) return [];
  
  return result[0].values.map(row => ({
    id: row[0],
    name: row[1] || `Simulation ${row[0]}`
  }));
}

// Initialize the app after database and simulation are loaded
function initializeApp() {
  // Load data from database
  edges = loadEdgesFromDB();
  densities = loadRoadDataFromDB();
  globalData = loadGlobalDataFromDB();

  console.log("Loaded edges:", edges.length);
  console.log("Loaded density timestamps:", densities.length);
  console.log("Loaded global data:", globalData.length);

  if (!edges.length) {
    alert("No edges found in database.");
    return;
  }

  if (!densities.length) {
    alert(`No road_data found for simulation ID ${selectedSimulationId}.`);
    return;
  }

  timeStamp = densities[0].datetime;

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

    let currentChartColumn = 'mean_density_vpk';

    // Initialize Chart
    if (globalData && globalData.length > 0) {
      const columns = Object.keys(globalData[0]).filter(k => k !== 'datetime');
      const selector = document.getElementById('chartColumnSelector');
      selector.innerHTML = '';
      columns.forEach(col => {
          const option = document.createElement('option');
          option.value = col;
          option.text = col;
          selector.appendChild(option);
      });
      
      if (columns.includes('mean_density_vpk')) {
          selector.value = 'mean_density_vpk';
      } else if (columns.length > 0) {
          selector.value = columns[0];
      }
      currentChartColumn = selector.value;

      selector.onchange = () => {
          currentChartColumn = selector.value;
          initChart();
          updateChart();
      };

      initChart();
      // document.querySelector('.chart-container').style.display = 'block';
    }

    function initChart() {
      const canvas = document.getElementById('densityChart');
      const ctx = canvas.getContext('2d');
      if (chart) chart.destroy();
      
      chart = new Chart(ctx, {
        type: 'line',
        data: {
          labels: globalData.map(d => formatTime(d.datetime)),
          datasets: [{
            label: currentChartColumn,
            data: globalData.map(d => d[currentChartColumn]),
            borderColor: 'blue',
            borderWidth: 1,
            pointRadius: 0,
            fill: false,
            tension: 0.1
          }, {
            label: 'Current Time',
            data: [], // Will be populated dynamically
            borderColor: 'red',
            backgroundColor: 'red',
            pointRadius: 5,
            pointHoverRadius: 7,
            showLine: false
          }]
        },
        options: {
          responsive: true,
          maintainAspectRatio: false,
          animation: {
              duration: 0
          },
          scales: {
            x: {
              display: true,
              title: {
                display: true,
                text: 'time'
              },
              ticks: {
                  display: false
              }
            },
            y: {
              beginAtZero: true,
              title: {
                display: true,
                text: currentChartColumn
              }
            }
          },
          plugins: {
              legend: {
                  display: true,
                  labels: {
                      boxWidth: 10
                  }
              }
          }
        }
      });

      // Add drag behavior
      let isDragging = false;

      const updateTimeFromEvent = (e) => {
        const points = chart.getElementsAtEventForMode(e, 'index', { intersect: false }, true);
        
        if (points.length) {
          const firstPoint = points[0];
          const index = firstPoint.index;
          
          // Update slider
          const timeSlider = document.getElementById('timeSlider');
          let currentDt = 300;
          if (densities.length > 1) {
            currentDt = Math.round((densities[1].datetime - densities[0].datetime) / 1000);
            if (currentDt <= 0) currentDt = 300;
          }
          
          timeSlider.value = index * currentDt;
          timeSlider.dispatchEvent(new Event('input'));
        }
      };

      canvas.onmousedown = (e) => {
        isDragging = true;
        updateTimeFromEvent(e);
      };

      canvas.onmousemove = (e) => {
        if (isDragging) {
          updateTimeFromEvent(e);
        }
      };

      canvas.onmouseup = () => {
        isDragging = false;
      };

      canvas.onmouseleave = () => {
        isDragging = false;
      };
    }

    function updateChart() {
        if (!chart || !globalData) return;
        
        // Find current data point index based on timeStamp
        const currentIndex = densities.findIndex(d => d.datetime.getTime() === timeStamp.getTime());
        
        if (currentIndex !== -1) {
            const pointData = new Array(globalData.length).fill(null);
            if (globalData[currentIndex]) {
                pointData[currentIndex] = globalData[currentIndex][currentChartColumn];
            }
            chart.data.datasets[1].data = pointData;
            chart.update('none');
        }
    }

    // Function to update edge positions, and color edges based on density
    function update() {
      // Update edge stroke width based on zoom level (handled in Canvas layer)
      // No need to update paths, Canvas layer handles it

      updateDensityVisualization();
      updateNodeHighlight();
      updateChart();
    }

    map.on("zoomend", update);
    update(); // Initial render

    // Update edge colors based on the current time step density data
    function updateDensityVisualization() {
      const currentDensityRow = densities.find(d => d.datetime.getTime() === timeStamp.getTime());
      if (!currentDensityRow) {
        console.error("No density data for time step:", timeStamp);
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
      canvasEdges.setDensities(currentDensities);
    }

    // Set up the time slider based on the density data's maximum time value
    const timeSlider = document.getElementById('timeSlider');
    const timeLabel = document.getElementById('timeLabel');
    const playBtn = document.getElementById('playBtn');
    const fpsInput = document.getElementById('fpsInput');
    
    let isPlaying = false;
    let playInterval = null;

    // Dynamically determine dt from the first two density datapoints
    let dt = 300;
    if (densities.length > 1) {
      dt = Math.round((densities[1].datetime - densities[0].datetime) / 1000); // in seconds
      if (dt <= 0) dt = 300;
    }
    timeSlider.max = (densities.length - 1) * dt;
    timeSlider.step = dt;
    timeLabel.textContent = `${formatTime(timeStamp)}`;

    function togglePlay() {
      isPlaying = !isPlaying;
      playBtn.textContent = isPlaying ? '⏸' : '▶';
      
      if (isPlaying) {
        const fps = parseFloat(fpsInput.value) || 10;
        const interval = 1000 / fps;
        
        playInterval = setInterval(() => {
          let currentValue = parseInt(timeSlider.value);
          let maxValue = parseInt(timeSlider.max);
          
          if (currentValue >= maxValue) {
            currentValue = 0; // Loop back to start
          } else {
            currentValue += dt;
          }
          
          timeSlider.value = currentValue;
          // Trigger input event manually
          timeSlider.dispatchEvent(new Event('input'));
        }, interval);
      } else {
        clearInterval(playInterval);
        playInterval = null;
      }
    }

    playBtn.addEventListener('click', togglePlay);

    fpsInput.addEventListener('change', () => {
      if (isPlaying) {
        // Restart with new FPS
        togglePlay(); // Stop
        togglePlay(); // Start
      }
    });

    // Update the visualization when the slider value changes
    timeSlider.addEventListener('input', function() {
      const index = Math.floor(parseInt(timeSlider.value) / dt);
      timeStamp = densities[index].datetime;
      timeLabel.textContent = `${formatTime(timeStamp)}`;
      update();
      // Update edge info if an edge is selected
      if (highlightedEdge) {
        const edge = edges.find(e => e.id === highlightedEdge);
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
      const edgeAsSource = edges.find(e => e.source === id);
      const edgeAsTarget = edges.find(e => e.target === id);
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

    // Add Enter key support for edge search
    const edgeSearchInput = document.getElementById('edgeSearch');
    edgeSearchInput.addEventListener('keydown', (event) => {
      if (event.key === 'Enter') {
        event.preventDefault();
        edgeSearchBtn.click();
      }
    });

    // Add Enter key support for node search
    const nodeSearchInput = document.getElementById('nodeSearch');
    nodeSearchInput.addEventListener('keydown', (event) => {
      if (event.key === 'Enter') {
        event.preventDefault();
        nodeSearchBtn.click();
      }
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
      const currentEdge = edges.find(e => e.id === highlightedEdge);
      if (!currentEdge) return;
      
      // Find inverse edge: source == current target, target == current source
      const inverseEdge = edges.find(e => e.source === currentEdge.target && e.target === currentEdge.source);
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

  // Show UI elements
  document.querySelector('.slider-container').style.display = 'block';
  document.querySelector('.legend-container').style.display = 'block';
  if (globalData.length > 0) {
    document.querySelector('.chart-container').style.display = 'block';
  }
}

// Database loading and simulation selection via modal
document.addEventListener('DOMContentLoaded', () => {
  const modal = document.getElementById('db-modal');
  const dbFileInput = document.getElementById('dbFileInput');
  const loadDbBtn = document.getElementById('loadDbBtn');
  const dbStatus = document.getElementById('db-status');
  
  loadDbBtn.addEventListener('click', async () => {
    const file = dbFileInput.files[0];
    if (!file) {
      dbStatus.className = 'db-status error';
      dbStatus.textContent = 'Please select a database file.';
      return;
    }
    
    dbStatus.className = 'db-status loading';
    dbStatus.textContent = 'Loading database...';
    loadDbBtn.disabled = true;
    
    try {
      // Initialize sql.js
      const SQL = await initSqlJs({
        locateFile: file => `https://cdnjs.cloudflare.com/ajax/libs/sql.js/1.10.3/${file}`
      });
      
      // Read the file
      const arrayBuffer = await file.arrayBuffer();
      const uint8Array = new Uint8Array(arrayBuffer);
      
      // Open the database
      db = new SQL.Database(uint8Array);
      
      // Verify tables exist
      const tables = db.exec("SELECT name FROM sqlite_master WHERE type='table'");
      const tableNames = tables.length > 0 ? tables[0].values.map(r => r[0]) : [];
      
      if (!tableNames.includes('edges')) {
        throw new Error("Database missing 'edges' table");
      }
      if (!tableNames.includes('road_data')) {
        throw new Error("Database missing 'road_data' table");
      }
      if (!tableNames.includes('simulations')) {
        throw new Error("Database missing 'simulations' table");
      }
      
      // Get available simulations
      const simulations = getSimulations();
      
      if (simulations.length === 0) {
        throw new Error("No simulations found in database");
      }
      
      dbStatus.className = 'db-status success';
      dbStatus.textContent = `Database loaded! Found ${simulations.length} simulation(s).`;
      
      // Show simulation selector
      setTimeout(() => {
        showSimulationSelector(simulations);
      }, 500);
      
    } catch (error) {
      console.error('Database loading error:', error);
      dbStatus.className = 'db-status error';
      dbStatus.textContent = `Error: ${error.message}`;
      loadDbBtn.disabled = false;
    }
  });
  
  // Simulation selector function
  function showSimulationSelector(simulations) {
    const modalContent = document.querySelector('.modal-content');
    
    modalContent.innerHTML = `
      <h2>Select Simulation</h2>
      <p>Choose which simulation to visualize:</p>
      <div class="db-input-group">
        <select id="simulationSelector" style="width: 100%; padding: 10px; font-size: 16px; border: 2px solid #ccc; border-radius: 5px;">
          ${simulations.map(sim => `<option value="${sim.id}">${sim.name} (ID: ${sim.id})</option>`).join('')}
        </select>
      </div>
      <div id="db-status" class="db-status"></div>
      <button id="loadSimBtn" class="load-db-btn">Load Simulation</button>
    `;
    
    document.getElementById('loadSimBtn').addEventListener('click', () => {
      selectedSimulationId = parseInt(document.getElementById('simulationSelector').value);
      document.getElementById('db-modal').classList.add('hidden');
      initializeApp();
    });
  }
});
