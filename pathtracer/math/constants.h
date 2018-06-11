//-------------------------------------------------------------------------------------------------
// Toy path tracer
//-------------------------------------------------------------------------------------------------
// Based on the minibook 'Raytracing in one weekend' and Aras P.'s series: Daily pathtracer
// https://aras-p.info/blog/
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
#pragma once

namespace math {
	// Values of math constants taken from
	// http://www.exploringbinary.com/pi-and-e-in-binary/
	// or computed as the binary approximation, then translated to decimal.
	// Which is more precise than rounding in decimal

	//----------------------------------------------
	template<typename T_>
	struct Constants {
	};

	//----------------------------------------------
	template<>
	struct Constants<float> {
		static constexpr float e		= 2.71828174591064453125f;
		static constexpr float pi		= 3.1415927410125732421875f;
		static constexpr float twoPi	= 6.2831852436065673828125f;
		static constexpr float halfPi	= 1.57079637050628662109375f;
	};

	//----------------------------------------------
	template<>
	struct Constants<double> {
		static constexpr double e		= 2.718281828459045090795598298427648842334747314453125;
		static constexpr double pi		= 3.141592653589793115997963468544185161590576171875;
		static constexpr double twoPi	= 6.283185307179586676085136787150986492633819580078125;
		static constexpr double halfPi	= 1.5707963267948965579989817342720925807952880859375;
	};

	static constexpr float HalfPi = Constants<float>::halfPi;
	static constexpr float Pi = Constants<float>::pi;
	static constexpr float TwoPi = Constants<float>::twoPi;
}