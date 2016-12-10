var num_az_steps  = 50;
var num_pol_steps = 30;
var num_f_steps = 40;

var screen_width  = 1280;
var screen_height =  750;
var max_force = 40;

collectEvents = document.getElementById('alerts_wrapper');
x = document.getElementById('x');
y = document.getElementById('y');
// f = document.getElementById('f');

collectEvents.addEventListener('click', function(e) {
  console.log("Cursor at: " + e.layerX + ", " + e.layerY);
  x.value = Math.floor((e.layerX*num_az_steps)/screen_width);
  y.value = num_pol_steps - Math.floor((e.layerY*num_pol_steps)/screen_height);
});
