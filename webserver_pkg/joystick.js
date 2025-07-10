var linear_x = 0.3
var angular_z = 0.3
var delta_x = 0.03
var delta_z = 0.03
var allowtorun = 0
var direction = 1

var app = new Vue({
    el: '#app',
    data: {
      connected: false,
      ros: null,
      ws_address: 'ws://192.168.1.43:9090',
      topic: null,
      message: null
  
    },
  
    methods: {

        connect: function(){
            console.log('Did you connect to rosbridge?');
            
            this.ros = new ROSLIB.Ros({
              url : this.ws_address
              // console.log(this.ws_address)
            });

            this.ros.on('connection', () => {
              console.log('Connected!');
              this.connected = true;
            });
          
            this.ros.on('error', (error) => {
              console.log('Error connecting to websocket server: ', error);
            });
          
            this.ros.on('close', () => {
              console.log('Connection to websocket server closed.');
              this.connected = false;
            })
        },
  
        disconnect: function(){
          this.ros.close();
        },
  
        setTopic: function(){
          this.topic = new ROSLIB.Topic({
            ros : this.ros,
            name : 'cmd_vel',
            messageType : 'geometry_msgs/Twist'
          })
        },
  
        goForward: function(){
          allowtorun = 1;
          direction = 1;
          this.message = new ROSLIB.Message({
            linear : {
              x : linear_x,
              y : 0,
              z : 0
            },
            angular : {
              x : 0,
              y : 0,
              z : 0
            }
          });
          this.setTopic();
          this.topic.publish(this.message);
        },
  
        goLeft: function(){
          allowtorun = 1;
          direction = 2;
          this.message = new ROSLIB.Message({
            linear : {
              x : 0,
              y : 0,
              z : 0
            },
            angular : {
              x : 0,
              y : 0,
              z : angular_z
            }
          });
          this.setTopic();
          this.topic.publish(this.message);
        },
        
        goRight: function(){
          allowtorun = 1;
          direction = 3;
          this.message = new ROSLIB.Message({
            linear : {
              x : 0,
              y : 0,
              z : 0
            },
            angular : {
              x : 0,
              y : 0,
              z : -angular_z
            }
          });
          this.setTopic();
          this.topic.publish(this.message);
        },
        
        goBack: function(){
          allowtorun = 1;
          direction = 4;
          this.message = new ROSLIB.Message({
            linear : {
              x : -linear_x,
              y : 0,
              z : 0
            },
            angular : {
              x : 0,
              y : 0,
              z : 0
            }
          });
          this.setTopic();
          this.topic.publish(this.message);
        },
  
        goStop: function(){
          allowtorun = 0;
          direction = 0;
          this.message = new ROSLIB.Message({
            linear : {
              x : 0,
              y : 0,
              z : 0
            },
            angular : {
              x : 0,
              y : 0,
              z : 0
            }
          });
          this.setTopic();
          this.topic.publish(this.message);
        },

        Upspeed: function(){
          linear_x = linear_x + delta_x
          angular_z = angular_z + delta_z
          if(linear_x >= 0.6){
            linear_x = 0.6
          }
          if(angular_z >= 0.6){
            angular_z = 0.6
          }
          document.getElementById("linear_x").innerHTML = Number(linear_x.toFixed(3));
          document.getElementById("angular_z").innerHTML = Number(angular_z.toFixed(3));
          // console.log('Speed of robot', linear_x, " ", angular_z);
          if(direction == 1){
            this.goForward();
          }
          else if(direction == 2){
            this.goLeft();
          }
          else if(direction == 3){
            this.goRight();
          }
          else if(direction == 4){
            this.goBack();
          }
          else{
            this.goStop();
          }

        },

        Downspeed: function(){
          // this.goStop();
          linear_x = linear_x - delta_x
          angular_z = angular_z - delta_z
          if(linear_x <= 0.03){
            linear_x = 0.03
          }
          if(angular_z <= 0.03){
            angular_z = 0.03
          }
          document.getElementById("linear_x").innerHTML = Number(linear_x.toFixed(3));
          document.getElementById("angular_z").innerHTML = Number(angular_z.toFixed(3));
          // console.log('Speed of robot', linear_x, " ", angular_z);
          if(direction == 1){
            this.goForward();
          }
          else if(direction == 2){
            this.goLeft();
          }
          else if(direction == 3){
            this.goRight();
          }
          else if(direction == 4){
            this.goBack();
          }
          else{
            this.goStop();
          }
        },

      }
  }
)
