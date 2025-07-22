$fn=64;

// Parameters
screw_diameter=3.2; // Diameter of screw holes
screw_head_diameter=6; // Diameter of screw head recess
corner_cylinder_diameter=5; // Diameter of corner cylinders
corner_cylinder_height=14.5; // Height of corner cylinders

taper_height=2; // Taper height on corner cylinders
base_thickness=2; // Thickness of the bottom plate
pcb_dimensions=[72,42,1]; // PCB dimensions [length, width, thickness]
lcd_dimensions=[44.5,40.5,1.5]; // LCD dimensions [length, width, thickness]
keyboard_button_diameter=5.8; // Diameter of keyboard buttons
keyboard_button_spacing=10; // Spacing between buttons
offset=0.2; // Offset

// Rendering
full_assembly();


// Main assembly
module full_assembly()
{
  // Assembles all enclosure components
  enclosure_cap();
  translate([0,0,-35]) enclosure_bottom(); 
  translate([-33,-20,5.5]) color("Blue") lcd_module();
  translate([-2,1,-8.5]) color("Green") pcb_module();     
  translate([-25,2,-29]) rotate([0,0,90]) PowerPcb();   
  translate([15,2,-28.85]) rotate([0,0,90]) PSU();    
  translate([-3.5,2,-11]) pad();      
}


module pad(thickness=4)
{
  difference()
  {  
  difference() 
  {
    cube([88, 47, thickness], center=true);
      for (x = [-42.5, 42.5], y = [-21, 21])
        translate([x, y, 0]) cylinder(h=10, d=12, center=true);
    hull() 
    {
      for (y = [-6, 6])
        translate([-38, y, 0]) cylinder(h=10, d=2, center=true);
      translate([-44, 0, 0]) cube([1, 14, 10], center=true);
    }
  }
  
  delta=1;
  translate([0, 0, -1]) difference() 
  {
    cube([88-delta*2, 47-delta*2, thickness], center=true);
      for (x = [-42.5+delta, 42.5-delta], y = [-21+delta, 21-delta])
        translate([x, y, 0]) cylinder(h=10, d=12+delta, center=true);
    hull() 
    {
      for (y = [-6-delta, 6+delta])
        translate([-38+delta, y, 0]) cylinder(h=10, d=2, center=true);
      translate([-44, 0, 0]) cube([1, 14+delta*2, 10], center=true);
    }
  }  
  }
  
}



module PSU()
{
  cube([43, 33, 1.3],center=true);   // PCB
  translate([0,0, 7])cube([41, 29, 15.3],center=true);   // Components      
}    

module PowerPcb()
{
   cube([48, 26, 1],center=true);   // PCB
   // Connector 
   translate([-(48-7.7)/2+2,0, (6.8+1)/2])
   { 
     difference()
     {  
       hull()
       {  
         cube([7.7,  20.5, 6.8],center=true);       
         translate([-0.5,0,2 ])cube([6,  20.5, 10.8],center=true);          
       }
       for(i=[-1:2]) translate([-0.5, 2.54*2*i-2.54, 7]) cylinder(h=3,d=3.2,center=true);
       for(i=[-1:2]) translate([-4, 2.54*2*i-2.54, -1.25]) rotate([0,90,0]) cylinder(h=3,d=3.2,center=true);  
           
     }

   }
    
    
}


// Top cover
module enclosure_cap()
{
  difference()
  {
    union()
    {
      // Main body
     difference()
     {
       hull()  corner_cylinders(corner_cylinder_diameter,corner_cylinder_height,taper_height);
       translate([-33,-20, 5.4]) lcd_and_keyboard_hole();         
       translate([-11,0,6.5]) cube([35,32,20],center=true);
       translate([-4.5,2, -7.75]) cube([89,48,26.5],center=true);          
    }
    // Bottom rim
    difference()
    {
      hull() corner_cylinders(2,5,0,-10);
      translate([-4.5,2,0]) cube([89,48,30],center=true); 
    }
    // PCB supports
    pcb_supports();
    // Screw mounts
    translate([0,0,2]) screw_mounts(8,18);
  }
  translate([8,26,-4]) cube([18,20,6],center=true); // UART hole  
  screw_holes();// Screw holes
 }
 // Print supports 
  translate([8,27.5,-4]) cube([0.4,3,6],center=true); 
  translate([13,27.5,-4]) cube([0.4,3,6],center=true);  
  translate([3,27.5,-4]) cube([0.4,3,6],center=true); 
}

// Corner cylinders
module corner_cylinders(diameter,height,taper,z_offset=-5,delta=0)
{
  // Creates corner cylinders for hull
  positions=[[43+delta,28+delta,z_offset],[43+delta,-24-delta,z_offset],[-50-delta,-24-delta,z_offset],[-50-delta,28+delta,z_offset]];
  for(pos=positions)
  {
    translate(pos)
    {
      cylinder(h=height,d=diameter,center=false);
      if(taper>0) translate([0,0,height]) cylinder(h=taper,d1=diameter,d2=diameter/2,center=false);
    }
  }
}

// PCB supports
module pcb_supports()
{
  // Supports for fixing the PCB
  translate([-40, 18,-1]) cube([6,20,14],center=true);
  translate([ 34, 20,-1]) cube([6,12,14],center=true);    
  translate([-40, -16,-1]) cube([6,12,14],center=true);    
  translate([-42,-20,-1]) cube([16,6,14],center=true); 
  translate([-42,23, -1]) cube([16,8,14],center=true);
  translate([28,23,-1]) cube([20,8,14],center=true);
}

// Screw mounts
module screw_mounts(diameter,height,z_offset=-3)
{
  // Cylinders for screw mounting
  positions=[[-46,23,z_offset],[-46,-19,z_offset],[39,23,z_offset],[39,-19,z_offset]];
  for(pos=positions) translate(pos) cylinder(h=height,d=diameter,center=true);
}

// Screw holes
module screw_holes(hex=false)
{
  // Holes with countersinking (optionally hexagonal)
  positions=[[-46,23,-3],[-46,-19,-3],[39,23,-3],[39,-19,-3]];
  for(pos=positions)
  {
    translate(pos)
    {
      cylinder(h=30,d=screw_diameter,center=true);
      translate([0,0,12]) cylinder(h=6,d=screw_head_diameter,center=true,$fn=hex?6:64);
      translate([0,0,8.5]) cylinder(h=1,d1=screw_diameter,d2=screw_head_diameter,center=true);
    }
  }
}

// Cutout for LCD and keyboard
module lcd_and_keyboard_hole()
{
  lcd_module();
  translate([54.5,19.5,1.3]) rotate([0,0,45]) keyboard_buttons();
}

// Keyboard buttons
module keyboard_buttons()
{
  // Cutouts for buttons and central square
  positions=[[5,5,0.8],[5,-5,0.8],[-5,-5,0.8],[-5,5,0.8]];
  for(pos=positions)
  {
    translate(pos)
    {
      translate([0,0,-0.1]) cylinder(h=5,d=keyboard_button_diameter+offset,center=false);
      translate([0,0,3]) cylinder(h=3,d1=keyboard_button_diameter+offset,d2=12.5,center=false);
    }
  }
  translate([0,0,0.4]) intersection()
  {
    cube([20,20,0.8],center=true);
    cylinder(h=1,d=26,center=true);
  }
}

// LCD display
module lcd_module()
{
  // LCD model with mounting area
  cube(lcd_dimensions,center=false);
  cube([67.5,33,1.5],center=false);
  translate([1.5,1.5,-0.4]) cube([40.5,37,6],center=false);
}

// Printed circuit board
module pcb_module()
{
  cube(pcb_dimensions,center=true);
}

// Bottom cover
module enclosure_bottom()
{
  difference()
  {
    union()
    {
      // Main body
      difference()
      {
        hull() corner_cylinders(corner_cylinder_diameter,12+18,0,0);   // +18
        translate([-3.5,2,12]) cube([90,48,40],center=true); 
        hull() translate([0,0,6.9]) corner_cylinders(2,5.2,0,0+18, offset*1.3);
        translate([-25,2,24]) cube([28,50,40],center=true);  
      }
      // Bottom plate
      translate([-3,2,1]) cube([92,50,base_thickness],center=true); 
      // Screw mounts
      screw_mounts(8,6+18,3.9+18/2);
      // Septum
      translate([-6,2,11]) cube([3,54,18],center=true);       
      // Input fitting
      translate([-25,-26,8.5]) rotate([90,0,0]) hull()
      {
        translate([8,0,0]) cylinder(h=5,d=12,center=true);
        translate([-8,0,0]) cylinder(h=5,d=12,center=true);
        translate([0,-7.5,0]) cube([28,2,5],center=true);          
      } 
      
      // Fitting for NTC wires
      translate([-32,-26,19]) rotate([90,0,0]) hull()
      {
        translate([1.5,0,0]) cylinder(h=5,d=6,center=true);
        translate([-1.5,0,0]) cylinder(h=5,d=6,center=true);
        translate([1.5,0,0]) cylinder(h=1,d=8,center=true);
        translate([-1.5,0,0]) cylinder(h=1,d=8,center=true);          
      }        
      
    }
    
    translate([0,4,7]) rotate([180,0,0]) screw_holes(hex=true); // Screw holes
    wire_holes();  // Holes in the septum for wires
    // Holes for power wires
    translate([-25,-24,8.5]) rotate([90,0,0]) hull()
    {
      translate([8,0,0]) cylinder(h=20,d=8,center=true);
      translate([-8,0,0]) cylinder(h=20,d=8,center=true);
    }      
    // Holes for NTC wires
    translate([-32,-24,19]) rotate([90,0,0]) hull()
    {
      translate([1.5,0,0]) cylinder(h=20,d=3,center=true);
      translate([-1.5,0,0]) cylinder(h=20,d=3,center=true);
    }     
  }     
  
  // Print supports 
  translate([-30,-25.75,9])
  {  
    translate([0,0,0]) cube([0.4,5.5,11],center=true); 
    translate([5,0,0]) cube([0.4,5.5,11],center=true);  
    translate([10,0,0]) cube([0.4,5.5,11],center=true);   
  }
}

// Holes for wires in the septum
module wire_holes()
{
  for(y=[-16,-10,14,20])
  {
    translate([-6,y,7]) rotate([0,90,0]) hull()
    {
      translate([3,0,0]) cylinder(h=10,d=3,center=true);
      translate([-3,0,0]) cylinder(h=10,d=3,center=true);
    }
  }
}