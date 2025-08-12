const map = L.map('map').setView([20, 105], 5);

L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
  attribution: '© OpenStreetMap contributors'
}).addTo(map);

const drawnItems = new L.FeatureGroup().addTo(map);

const polygonColor = 'blue';
const circlePoint = 64;

document.addEventListener('DOMContentLoaded', function () {

  const drawControl = new L.Control.Draw({
    draw: {
      polygon: {
        shapeOptions: {
          color: polygonColor, 
          fillColor: polygonColor,     
          fillOpacity: 0.2
        }
      },
      circle: {
        shapeOptions: {
          color: polygonColor, 
          fillColor: polygonColor,     
          fillOpacity: 0.2
        }
      },
      polyline: false,
      marker: false,
      rectangle: false,
      circlemarker: false
    },
    edit: {
      featureGroup: drawnItems
    }
  });
  map.addControl(drawControl);

  // Handle draw
  map.on(L.Draw.Event.CREATED, function (e) {
    if (e.layerType === 'circle') {
      const { lat, lng } = e.layer.getLatLng();
      const radius = e.layer.getRadius();
      const distanceKm = (radius / 1000).toFixed(2);
      const points = [];

      // layer.bindPopup('${distanceKm} km').openPopup();

      for (let i = 0; i < circlePoint; i++) {
        const angle = (i / circlePoint) * 360;
        const point = computeOffset([lat, lng], radius, angle);
        points.push(point);
      }
      points.push(points[0]);

      console.log('🟢 Distance:');
      console.log(distanceKm);
      console.log('🟢 Circle with 65 points:');
      console.log(points);

      //show marker
      // const polygon = L.polygon(points, {
      //   color: polygonColor,
      //   fillColor: polygonColor,
      //   fillOpacity: 0.2
      // });
      // drawnItems.addLayer(polygon);

      drawnItems.addLayer(e.layer);
    }else if (e.layerType === 'polygon') {
      const latlngs = e.layer.getLatLngs()[0];
      const coords = latlngs.map(p => [p.lat, p.lng]);
      
      console.log('🔷 Polygon points:');
      console.log(coords);
      drawnItems.addLayer(e.layer);
    }else{
      drawnItems.addLayer(e.layer);
    }
    
  });

  // Open button
  L.Control.OpenGeoJSON = L.Control.extend({
    onAdd: function (map) {
      const container = L.DomUtil.create('div', 'leaflet-bar leaflet-control');

      const link = L.DomUtil.create('a', '', container);
      link.innerHTML = '📂';
      link.href = '#';
      link.title = 'Open GeoJSON';

      const input = L.DomUtil.create('input', '', container);
      input.type = 'file';
      input.accept = '.geojson,.json';
      input.style.display = 'none';

      L.DomEvent.on(link, 'click', function (e) {
        L.DomEvent.stopPropagation(e);
        L.DomEvent.preventDefault(e);
        input.click();
      });

      input.addEventListener('change', function (e) {
        const file = e.target.files[0];
        if (!file) return;

        const reader = new FileReader();
        reader.onload = function (event) {
          try {
            const geojson = JSON.parse(event.target.result);
            drawnItems.clearLayers();
            const layer = L.geoJSON(geojson, {
              onEachFeature: function (feature, layer) {
                drawnItems.addLayer(layer);
              },
              style: function (feature) {
                return {
                  color: polygonColor,
                  fillColor: polygonColor,
                  fillOpacity: 0.2
                };
              }
            });
            map.fitBounds(layer.getBounds());
          } catch (err) {
            alert("Invalid GeoJSON file.");
            console.error(err);
          }finally {
            e.target.value = "";
          }
        };
        reader.readAsText(file);
      });

      return container;
    },
    onRemove: function (map) {}
  });
  map.addControl(new L.Control.OpenGeoJSON({ position: 'topright' }));

  // Save Button
  L.Control.SaveGeoJSON = L.Control.extend({
    onAdd: function (map) {
      const container = L.DomUtil.create('div', 'leaflet-bar leaflet-control');

      const link = L.DomUtil.create('a', '', container);
      link.innerHTML = '💾';
      link.href = '#';
      link.title = 'Save to GeoJSON';

      L.DomEvent.on(link, 'click', L.DomEvent.stopPropagation)
                .on(link, 'click', L.DomEvent.preventDefault)
                .on(link, 'click', () => {
                  const features = [];

                  drawnItems.eachLayer(layer => {
                    if (layer instanceof L.Circle) {
                      features.push(circleToPolygon(layer, circlePoint));
                    } else {
                      features.push(layer.toGeoJSON());
                    }
                  });

                  const geojson = {
                    type: "FeatureCollection",
                    features: features
                  };

                  const blob = new Blob([JSON.stringify(geojson, null, 2)], { type: "application/json" });
                  const url = URL.createObjectURL(blob);

                  const a = document.createElement('a');
                  a.href = url;
                  a.download = 'map.geojson';
                  a.click();

                  URL.revokeObjectURL(url);
                });

      return container;
    },
    onRemove: function (map) {}
  });
  map.addControl(new L.Control.SaveGeoJSON({ position: 'topright' }));

  // Setting Button
  // L.Control.CircleSettings = L.Control.extend({
  //   onAdd: function (map) {
  //     const container = L.DomUtil.create('div', 'leaflet-bar leaflet-control');

  //     const link = L.DomUtil.create('a', '', container);
  //     link.innerHTML = '⚙️';
  //     link.href = '#';
  //     link.title = 'Draw Circle from Settings';

  //     L.DomEvent.on(link, 'click', L.DomEvent.stopPropagation)
  //               .on(link, 'click', L.DomEvent.preventDefault)
  //               .on(link, 'click', () => {
  //                 const latlngInput  = prompt("Enter center coordinate (lat, lon):");
  //                 const radiusKm = prompt("Enter radius (km):");

  //                 if (!latlngInput || !radiusKm) return;

  //                 const parts = latlngInput.split(',');
  //                 if (parts.length !== 2) {
  //                   alert("Invalid coordinate format. Use: lat, lng");
  //                   return;
  //                 }

  //                 const latNum = parseFloat(parts[0].trim());
  //                 const lngNum = parseFloat(parts[1].trim());
  //                 const radiusMeters = parseFloat(radiusKm) * 1000;

  //                 if (isNaN(latNum) || isNaN(lngNum) || isNaN(radiusMeters)) {
  //                   alert("Invalid input.");
  //                   return;
  //                 }

  //                 // Draw circle
  //                 const circle = L.circle([latNum, lngNum], {
  //                   radius: radiusMeters,
  //                   color: polygonColor,
  //                   fillColor: polygonColor,
  //                   fillOpacity: 0.2
  //                 });

  //                 drawnItems.addLayer(circle);

  //                 // Add 64 points around circle
  //                 for (let i = 0; i < circlePoint; i++) {
  //                   const angle = (i / circlePoint) * 360;
  //                   const [ptLat, ptLng] = computeOffset([latNum, lngNum], radiusMeters, angle);
  //                   L.circleMarker([ptLat, ptLng], {
  //                     radius: 2,
  //                     color: polygonColor,
  //                     fillColor: polygonColor,
  //                     fillOpacity: 1
  //                   }).addTo(drawnItems);
  //                 }

  //                 // Fit map to new circle
  //                 map.fitBounds(circle.getBounds());
  //               });

  //     return container;
  //   },
  //   onRemove: function () {}
  // });
  // map.addControl(new L.Control.CircleSettings({ position: 'topright' }));

  L.Control.CircleSettings = L.Control.extend({
    onAdd: function (map) {
      const container = L.DomUtil.create('div', 'leaflet-bar leaflet-control');
      const link = L.DomUtil.create('a', '', container);
      link.innerHTML = '⚙️';
      link.href = '#';
      link.title = 'Draw circle with center + radius';

      L.DomEvent.on(link, 'click', function (e) {
        L.DomEvent.stopPropagation(e);
        L.DomEvent.preventDefault(e);

        // Show settings form
        document.getElementById('settings-form').style.display = 'block';
      });

      return container;
    },
    onRemove: function (map) {}
  });
  map.addControl(new L.Control.CircleSettings({ position: 'topright' }));

  // Form handlers
  document.getElementById('applySettings').addEventListener('click', function () {
    const latlngStr = document.getElementById('centerInput').value;
    const radiusKm = parseFloat(document.getElementById('radiusInput').value);
    const parts = latlngStr.split(',');

    if (parts.length !== 2 || isNaN(radiusKm)) {
      alert("Please enter valid lat,lng and radius.");
      return;
    }

    const lat = parseFloat(parts[0].trim());
    const lng = parseFloat(parts[1].trim());
    const radius = radiusKm * 1000;

    if (isNaN(lat) || isNaN(lng)) {
      alert("Invalid coordinates.");
      return;
    }

    // Draw circle
    const circle = L.circle([lat, lng], {
      radius: radius,
      color: polygonColor,
      fillColor: polygonColor,
      fillOpacity: 0.2
    });
    drawnItems.addLayer(circle);

    // Draw 64 points on the edge
    for (let i = 0; i < circlePoint; i++) {
      const angle = (i / circlePoint) * 360;
      const [ptLat, ptLng] = computeOffset([lat, lng], radius, angle);
      L.circleMarker([ptLat, ptLng], {
        radius: 2,
        color: polygonColor,
        fillColor: polygonColor,
        fillOpacity: 1
      }).addTo(drawnItems);
    }

    map.fitBounds(circle.getBounds());

    // Hide form
    document.getElementById('settings-form').style.display = 'none';
    document.getElementById('centerInput').value = '';
    document.getElementById('radiusInput').value = '';
  });

  document.getElementById('cancelSettings').addEventListener('click', function () {
    document.getElementById('settings-form').style.display = 'none';
  });


  function initDrawControl(color) {
    if (drawControl) map.removeControl(drawControl);

    drawControl = new L.Control.Draw({
      draw: {
        polygon: {
          shapeOptions: {
            color: color,
            fillColor: color,
            fillOpacity: 0.2
          }
        },
        circle: {
          shapeOptions: {
            color: color,
            fillColor: color,
            fillOpacity: 0.2
          }
        },
        polyline: false,
        marker: false,
        rectangle: false,
        circlemarker: false
      },
      edit: {
        featureGroup: drawnItems
      }
    });

    map.addControl(drawControl);
  }

  function computeOffset([lat, lng], distanceMeters, angleDegrees) {
    const R = 6378137; // Earth radius in meters
    const rad = Math.PI / 180;
    const d = distanceMeters / R;

    const angleRad = angleDegrees * rad;
    const lat1 = lat * rad;
    const lng1 = lng * rad;

    const lat2 = Math.asin(Math.sin(lat1) * Math.cos(d) +
      Math.cos(lat1) * Math.sin(d) * Math.cos(angleRad));
    const lng2 = lng1 + Math.atan2(
      Math.sin(angleRad) * Math.sin(d) * Math.cos(lat1),
      Math.cos(d) - Math.sin(lat1) * Math.sin(lat2)
    );

    return [lat2 / rad, lng2 / rad];
  }

  function circleToPolygon(circle, points = circlePoint) {
    const center = circle.getLatLng();
    const radius = circle.getRadius(); // in meters
    const coords = [];

    for (let i = 0; i < points; i++) {
      const angle = (i / points) * 360;
      const [lat, lng] = computeOffset([center.lat, center.lng], radius, angle);
      coords.push([lng, lat]); // GeoJSON requires [lng, lat]
    }

    coords.push(coords[0]); // Close the polygon

    return {
      type: "Feature",
      properties: {},
      geometry: {
        type: "Polygon",
        coordinates: [coords]
      }
    };
  }


});
