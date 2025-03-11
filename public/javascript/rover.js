
// global variable for UI elements
let video = document.getElementById('videoFeed');
let map = document.getElementById('rockYardMap');
let poi1x = document.getElementById('poi1x');
let poi2x = document.getElementById('poi2x');
let poi3x = document.getElementById('poi3x');
let poi1y = document.getElementById('poi1y');
let poi2y = document.getElementById('poi2y');
let poi3y = document.getElementById('poi3y');

// Loads the rover video feed
function loadVideoFeed() {
    video.src = "http://192.168.51.194:8080/stream?topic=/camera/image_raw&type=ros_compressed";
}

function loadPOI(){

    fetch("../server_ip.txt")
    .then(response => response.text())
    .then(ip => fetch(`http://${ip}:14141`, {
        method: "POST",
        headers: {
            "Content-Type": "text/plain",
            "User-Agent": "rover_"  // Optional, to match the C request
        },
        body: "rover_ping=true" // Raw text body
    }))
    .then(response => response.text()) 
    .then(data => console.log("Success:", data))
    .catch(error => console.error("Error:", error));
   
    $.getJSON("../json_data/ROVER.json", function(data) {
        poi1x.innerText = (data.rover.poi_1_x).toFixed(5);
        poi2x.innerText = (data.rover.poi_2_x).toFixed(5);
        poi3x.innerText = (data.rover.poi_3_x).toFixed(5);
        poi1y.innerText = (data.rover.poi_1_y).toFixed(5);
        poi2y.innerText = (data.rover.poi_2_y).toFixed(5);
        poi3y.innerText = (data.rover.poi_3_y).toFixed(5);
    })
}

// Loads the rock yard maps
function loadMap() {
    map.src = "./images/rockYardMap-min.png";
}

//Runs on load of the page
function onload() {

    // Init all the variables
    video = document.getElementById('videoFeed');
    map = document.getElementById('rockYardMap');

    poi1x = document.getElementById('poi1x');
    poi1y = document.getElementById('poi1y');
    poi2x = document.getElementById('poi2x');
    poi2y = document.getElementById('poi2y');
    poi3x = document.getElementById('poi3x');
    poi3y = document.getElementById('poi3y');

    loadMap();
    loadPOI();

    // Continuously refreshes for the video feed
    setInterval(function() {
        loadVideoFeed();
    }, 1000);
}