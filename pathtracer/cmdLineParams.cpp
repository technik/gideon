//-------------------------------------------------------------------------------------------------
// Toy path tracer
//--------------------------------------------------------------------------------------------------
// Copyright 2018 Carmelo J Fdez-Aguera
// 
// Permission is hereby granted, free of charge, to any person obtaining a copy of this software
// and associated documentation files (the "Software"), to deal in the Software without restriction,
// including without limitation the rights to use, copy, modify, merge, publish, distribute,
// sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
// 
// The above copyright notice and this permission notice shall be included in all copies or
// substantial portions of the Software.
// 
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT
// NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
// NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM,
// DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.

#include "cmdLineParams.h"

using namespace std;

//--------------------------------------------------------------------------------------------------
CmdLineParams::CmdLineParams(int _argc, const char** _argv)
{
	vector<string> args(_argc);
	// Read all params
	int i = 0;
	for(auto& s : args)
		s = _argv[i++];
	i = 0;
	while(i < _argc)
		i += process(args, i);
}

//--------------------------------------------------------------------------------------------------
int CmdLineParams::process(const vector<string>& args, int i)
{
	auto& arg = args[i];
	if(arg == "-bg") {
		background = args[i+1];
		return 2;
	}
	if(arg == "-scene")
	{
		scene = args[i+1];
		return 2;
	}
	if(arg == "-solid")
	{
		overrideMaterials = true;
		return 1;
	}
	if(arg == "-o")
	{
		output = args[i+1];
		return 2;
	}
	if(arg == "-s") // samples per pixel
	{
		ns = atoi(args[i+1].c_str());
		return 2;
	}
	if(arg == "-w")
	{
		sx = atoi(args[i+1].c_str());
		return 2;
	}
	if(arg == "-h")
	{
		sy = atoi(args[i+1].c_str());
		return 2;
	}
	if(arg == "-fov")
	{
		fov = (float)atof(args[i+1].c_str());
		return 2;
	}
	if(arg == "-tile")
	{
		tileSize = atoi(args[i+1].c_str());
		return 2;
	}
	if(arg == "-fullHD")
	{
		sx = 1920;
		sy = 1080;
		return 1;
	}
	if(arg == "-spherical")
	{
		sphericalRender = true;
		return 1;
	}
	return 1;
}