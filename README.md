# Team-6---Robot-Summer



bottom -262
top 0











Linear arm has 160 encoder ticks between the limit switches. extended is -160, closed is 0


Elevator has 515 encoder ticks between the limit switches. Top is 0, bottom is -515.



elevator calib:

bottom limit: -15
claw heighy: ~241 to 267
forklift height: ~bottom of limit switch 



Tasks to do:

PID control of rod with limit switches (needed for:)
    Elevator system movement
    Plate serving movement
    rack and pinion 
    Lazy susan rotation code (the same type of abstraction. I think identical code as above)


Claw servo integration with limit switches

DONE: drive PID tuning
    find better suited motors


Determine communcation data structure & what information to send

PID tuning for IR beacon sensor

start to combine code components 
