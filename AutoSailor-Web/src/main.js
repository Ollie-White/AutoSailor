mapboxgl.accessToken = import.meta.env.VITE_MAPBOX_ACCESS_TOKEN; //Load in token for mapbox usage

//Set bounds for how far users can move the map around
const boundWidth = 0.02
const boundHeight = 0.01

//Center point of the map
const center_lat = parseFloat(import.meta.env.VITE_MAP_CENTER_LAT)
const center_long = parseFloat(import.meta.env.VITE_MAP_CENTER_LONG)

//The edges of your overlay image to align it with the map
const lake_left_edge = parseFloat(import.meta.env.VITE_MAP_EDGE_LEFT)
const lake_right_edge = parseFloat(import.meta.env.VITE_MAP_EDGE_RIGHT)
const lake_top_edge = parseFloat(import.meta.env.VITE_MAP_EDGE_TOP)
const lake_bottom_edge = parseFloat(import.meta.env.VITE_MAP_EDGE_BOTTOM)

const bounds = [
  [center_long - boundWidth, center_lat - boundHeight], // Southwest (minLng, minLat)
  [center_long + boundWidth, center_lat + boundHeight]  // Northeast (maxLng, maxLat)
];

const map = new mapboxgl.Map({
    container: 'map', // container ID
    center: [center_long, center_lat],
    zoom: 13,
    minZoom: 13, //Stop the users from being able to zoom out too much
    style:  'mapbox://styles/olliewhite/cm81gs1o800l801r38errhman', //More visible map for aligning the image overlay //'mapbox://styles/olliewhite/cldszddhu005401mg3ijl1669',
    maxBounds: bounds,
    dragRotate: false,
    touchRotate: false
});

// Create a marker for the sailboat
let sailboatMarker = new mapboxgl.Marker({
    color: 'red'
  })
  .setLngLat([center_long, center_lat])  // Starting position (set later with real data)
  .addTo(map);

  let destinationMarker = new mapboxgl.Marker()
  .setLngLat([center_long, center_lat])  // Starting position (set later with real data)

// Establish a socket connection to the Flask API
const socket = io.connect(import.meta.env.VITE_API_URL);  // Replace with your API URL if different

map.on('load', () => {
  map.addSource('lake', {
      'type': 'image',
      'url': '/LakeOutline2.PNG',
      'coordinates': [
        [lake_left_edge, lake_top_edge], // NW (Top-left)
        [lake_right_edge, lake_top_edge], // NE (Top-right)
        [lake_right_edge, lake_bottom_edge], // SE (Bottom-right)
        [lake_left_edge, lake_bottom_edge]  // SW (Bottom-left)
      ]
  });

  map.addLayer({
      id: 'lake-layer',
      'type': 'raster',
      'source': 'lake',
      'paint': {
          'raster-fade-duration': 0,
          //'raster-opacity': 0.5 //Used for making sure your coordinates align properly 
      }
  });
}); 

//Add marker and output the coordinates for sending to the boat later.
map.on('click', (event) => {
  const coords = event.lngLat;

  destinationMarker.remove();
  destinationMarker = new mapboxgl.Marker()
    .setLngLat([coords.lng, coords.lat])
    .addTo(map);

  console.log(`Clicked at: ${coords.lng}, ${coords.lat}`); 

  const destination = {
    lat: coords.lat,
    lng: coords.lng
  }

  console.log(destination)

  // Send API call to navigate the boat to the clicked point
  fetch(import.meta.env.VITE_API_URL+'/navigateBoatToPoint', {
    method: 'POST',
    headers: {
      'Content-Type': 'application/json',
    },
    body: JSON.stringify({
      destination: destination
    }),
  })
  .then(response => {
    if (response.ok) {
      return response.json();
    } else {
      throw new Error(response.json())
    }
  })
  .then(data => {

    console.log('Navigation response:', data);
    destinationMarker.remove();
    destinationMarker = new mapboxgl.Marker({color: 'green'})
      .setLngLat([coords.lng, coords.lat])
      .addTo(map);
    
    let waypoints = []
    for (let i = 0; i < data.waypoints.length; i++) {
      waypoints.push([data.waypoints[i][1], data.waypoints[i][0]])
    }
    console.log('waypoints:', waypoints)

    if (map.getLayer('route-line')) {
      map.removeLayer('route-line'); // Remove the layer
    }
    if (map.getSource('route')) {
        map.removeSource('route'); // Remove the source
    }

    // Add a source for the line
    map.addSource('route', {
        'type': 'geojson',
        'data': {
            'type': 'Feature',
            'properties': {},
            'geometry': {
                'type': 'LineString',
                'coordinates': waypoints
            }
        }
    });

    // Add a layer to style the line
    map.addLayer({
        'id': 'route-line',
        'type': 'line',
        'source': 'route',
        'layout': {
            'line-join': 'round',
            'line-cap': 'round'
        },
        'paint': {
            'line-color': 'purple',  // Red color
            'line-width': 2,        // Line thickness
            'line-dasharray': [2, 2]
        }
    });
    
    // You can add user feedback here, e.g., a toast notification
  })
  .catch(error => {
    console.error('Error sending navigation request:', error);
    destinationMarker.remove();
    // You can add error handling here, e.g., showing an error message to the user
  });
});

// Listen for position updates from the Flask backend
socket.on('position_update', (data) => {
  //console.log('Received position update:', data);

  if (data.lon !== undefined && data.lat !== undefined) {
    //console.log('Updating marker position to:', data.lon, data.lat);

    // Update marker position
    sailboatMarker.setLngLat([data.lon, data.lat]);
  } else {
    console.error('Invalid data received:', data);
  }
});

socket.on('error', (error) => {
  console.error('Socket error:', error);
});

socket.on('connect', (data) => {
  console.error('socket connected: ', data);
});