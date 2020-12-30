# a-star

### A simple implementation of A* Search

I wrote this A* implementation for a lecture I gave on the algorithm for my Robotics and 3D Printing class, which I took in Spring of 2020. While preparing for the lecture, I decided that it would be best to give a practical example of how A* works.

As this was a simple lecture demonstration, the matrix to traverse, as well as the start and end points, were hardcoded.

I actually discovered during the presentation that I had neglected to discard walls while discovering neighbor nodes, which caused paths to phase through walls. Oops. That kind of stuff happens when you are trying to write code too quickly without enough testing. I just rolled with it, since I was able to fix the problem and continue.
