var lastTime = new Date().getTime();
var trans_x = 50;
var trans_y = 50;
var goal_x = 0;
var goal_y = 50;
var goal_reached = true;
var frequency = 15;

const ros_ip = "127.0.0.1"
const ros_port = 9090


// Intervall function
setInterval(random_eyes, 1000/frequency)

function random_eyes(){
  var currentTime = new Date().getTime();
  var lastTime_sec = Math.floor(lastTime/1000);
  var currentTime_sec = Math.floor(currentTime/1000);
  if((currentTime_sec - lastTime_sec) > 1.5){
    if(goal_reached){
      var plusOrMinus = Math.random() < 0.5 ? -1 : 1;
      goal_x = plusOrMinus * getRandomInt(50);
      goal_y = getRandomInt(100);
      goal_reached = false;
    }
    else{
      distance_x = goal_x - trans_x;
      distance_y = goal_y - trans_y;
      if(-2 < distance_x && distance_x < 2 && -2 < distance_y && distance_y < 2){
        goal_reached = true;
      }
      if(distance_x > 0){
        step_x = 1*(distance_x/10);
      }
      else if(distance_x < 0){
        step_x = 1*(distance_x/10);
      }
      else{
        step_x = 0
      }
      if(distance_y > 0){
        step_y = 1*(distance_y/10);
      }
      else if(distance_y < 0){
        step_y = 1*(distance_y/10);
      }
      else{
        step_y = 0;
      }
      set_eyeballs(trans_x + step_x, trans_y + step_y);
    }
  }
}

function getRandomInt(max) {
  return Math.floor(Math.random() * Math.floor(max));
}


// Connecting to ROS
// -----------------

var ros = new ROSLIB.Ros();

ros.on('connection', function() {
  console.log('Connected to websocket server.');
});

ros.on('error', function(error) {
  console.log('Error connecting to websocket server: ', error);
  setTimeout(function() {
    connect();
  }, 1000);
});

ros.on('close', function() {
  console.log('Connection to websocket server closed.');
  setTimeout(function() {
    connect();
  }, 1000);
});

function connect(){
  ros.connect('ws://' + ros_ip + ':' + ros_port);
}

connect();

// Subscribing to Topics
// ----------------------
var listener_image = new ROSLIB.Topic({
  ros : ros,
  name : '/facedetection_image/compressed',
  messageType : 'sensor_msgs/CompressedImage'
});

listener_image.subscribe(function(message){
  console.log('Received message on ' + listener.name + ': ' + message.header);
  document.getElementById('facedetection').src = "data:image/jpg;base64," + message.data;
});


var listener = new ROSLIB.Topic({
  ros : ros,
  name : '/facedetection',
  messageType : 'geometry_msgs/Point'
});

listener.subscribe(function(message) {
  var currentTime = new Date().getTime();
  if(message.x != 0.0 || message.y != 0.0){
    lastTime = currentTime;
    set_eyeballs((-message.x*100+50), (message.y*100+20));
  }
});

function set_eyeballs(x, y){
  trans_x = x
  trans_y = y

  var eyeballs = document.getElementsByClassName("eyeball");
  [].forEach.call(eyeballs, function(eye){
    eye.style.transform = "translate(" + x + "%," + y + "%)";
  });
}
