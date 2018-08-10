# Gideon

Pathtracer pet project. Follow the interesting bits of its development in [my blog](https://technik90.blogspot.com/).
The initial idea was largely based on the book [Ray Tracing in One Weekend](http://in1weekend.blogspot.lt/) by Peter Shirley
and the [Daily Pathtracer](http://aras-p.info/blog/2018/03/28/Daily-Pathtracer-Part-0-Intro/) series by Aras Pranckevičius.

* [Part 1: Basic Jobs System](https://technik90.blogspot.com/2018/06/the-other-pathtracer-basic-job-system.html)
* [Part 2: The Triangle](https://technik90.blogspot.com/2018/06/the-other-pathtracer-2-triangle.html)
* [Part 3: Complex Scenes](https://technik90.blogspot.com/2018/06/the-other-pathtracer-3-complex-scenes.html)
* [Part 4: Optimizing AABB-Ray intersections](https://technik90.blogspot.com/2018/06/the-other-pathtracer-4-optimizing-aabb.html)
* [Part 5: Optimizing Ray-Triangle intersections](http://technik90.blogspot.com/2018/08/the-other-pathtracer-5-optimizing.html)

![Screenshot](/screenshot.png?raw=true)

## Current features

* Basic loading of [gltf](https://github.com/KhronosGroup/glTF/tree/master/specification/2.0) scenes
* HDR Background in .hdr format

## Libraries

* Currently using the following libraries, all included under the 'include' folder:
  * [fx/gltf](https://github.com/jessey-git/fx-gltf)
  * [nlohmann's json](https://github.com/nlohmann/json)
  * a few [stb](https://github.com/nothings/stb) single header libraries by Sean T. Barrett

## Building

Straightforward CMake build. I test on windows only, so other platforms may fail miserably.