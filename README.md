# marblemaze
Robotic Marble Maze Game

![Maze with ball](images/mazeball.png)

## Completed!
See [the YouTube Video](https://www.youtube.com/watch?v=sxEOGW4oePQ).

## Lessons Learned
The biggest challenges with this maze were getting the camera to track the ball reliably and getting the ball controller to work reliably.

* Wood is not the best choice of material for the base (or the walls). The small variations in the wood cause the marble to "stick" more than it would if it was on a smoother surface such as acrylic.
* The walls, while dark, actually are more red than black when viewed from a camera. This made it hard to segment them out cleanly. Painting the edges would be better.
* The marble should be a solid color. I was originally intending to use a steely, but it has too much glare to pick up well with a webcam.
* The webcam needs to have a fairly high refresh rate. Originally I was using an older camera which only refreshed at 10-15fps which was too slow to control the ball. A 30fps webcam was much better, though 60fps would be ideal.
* Having the servos mounted on a tilt-tilt gimbal worked ok, but small variations in the servo command led to high variations in the angle of the platform. It would be much better to have the servos mounted on the base with rods going to the edges of the platform.
* Because of the small range of motion used, I had to borrow some expensive digital servos. This performed very well, but I think cheaper servos would work fine if they had a larger range of motion.
* Having only some interchangeable walls wasn't that exciting. I think all of them should be interchangeable.
* It was necessary to tape paper around the edge of the maze and around the base to get the outside walls to segment consistently. A better option might be a flourescent dot on each corner, or maybe using black for the maze and base and use white for the wall edges.
* We relied on using the outside walls to do a perspective transform of the maze so the solver always saw it as square. Ths added a lot of noise to the ball localization. I'm wondering if it is possible to do something else.
* Tuning on a flat plate was very helpful
* The control might be improved by considering hybrid dynamics and the rolling dynamics of the ball.



