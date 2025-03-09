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
    style: 'mapbox://styles/olliewhite/cm81gs1o800l801r38errhman', // 'mapbox://styles/olliewhite/cldszddhu005401mg3ijl1669' //More visible map for aligning the image overlay
    maxBounds: bounds,
    dragRotate: false,
    touchRotate: false
});

map.on('load', () => {
  map.addSource('lake', {
      'type': 'image',
      'url': '/LakeOutline.PNG',
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
          // 'raster-opacity': 0.5 //Used for making sure your coordinates align properly 
      }
  });
}); 

//Add marker and output the coordinates for sending to the boat later.
map.on('click', (event) => {
  const coords = event.lngLat;

  new mapboxgl.Marker()
      .setLngLat([coords.lng, coords.lat])
      .addTo(map);

  console.log(`Clicked at: ${coords.lng}, ${coords.lat}`);
});