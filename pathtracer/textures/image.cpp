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
#include "image.h"

#define STBI_MSC_SECURE_CRT
#define STB_IMAGE_IMPLEMENTATION
#include <stb_image.h>
#define STB_IMAGE_WRITE_IMPLEMENTATION
#include <stb_image_write.h>

#include <algorithm>

Image::Image(const char* fileName)
{
	int nComponents;
	int isx, isy;
	auto rawData = stbi_loadf(fileName, &isx, &isy, &nComponents, 3);
	assert(isx >= 0 && isy >= 0);
	sx = size_t(isx);
	sy = size_t(isy);
	mData = data_ptr(reinterpret_cast<math::Vec3f*>(rawData), stbi_image_free);
}

Image::Image(size_t nx, size_t ny)
	: sx(nx)
	, sy(ny)
{
	mData = data_ptr(new math::Vec3f[nx * ny], [](void* x) { delete[] reinterpret_cast<math::Vec3f*>(x); });
}

void Image::saveAsSRGB(const char* fileName) const
{
	std::vector<uint8_t> tmpBuffer;
	const auto nPixels = area();
	tmpBuffer.reserve(nPixels);
	for (size_t i = 0; i < nPixels; ++i)
	{
		auto& c = mData[i];
		tmpBuffer.push_back(floatToByteColor(c.x()));
		tmpBuffer.push_back(floatToByteColor(c.y()));
		tmpBuffer.push_back(floatToByteColor(c.z()));
	}

	const int rowStride = int(3 * sx);
	stbi_write_png(fileName, (int)sx, (int)sy, 3, tmpBuffer.data(), rowStride);
}

void Image::saveAsLinearRGB(const char* fileName) const
{
	std::vector<uint8_t> tmpBuffer;
	const auto nPixels = area();
	tmpBuffer.reserve(nPixels);
	for (size_t i = 0; i < nPixels; ++i)
	{
		auto& c = mData[i];
		tmpBuffer.push_back(floatToLinearByteColor(c.x()));
		tmpBuffer.push_back(floatToLinearByteColor(c.y()));
		tmpBuffer.push_back(floatToLinearByteColor(c.z()));
	}

	const int rowStride = int(3 * sx);
	stbi_write_png(fileName, (int)sx, (int)sy, 3, tmpBuffer.data(), rowStride);
}

uint8_t Image::floatToByteColor(float value)
{
	auto clampedVal = std::clamp(value, 0.f, 1.f);
	auto sRGBVal = std::pow(clampedVal, 1.f / 2.23f);
	return uint8_t(sRGBVal * 255);
}

uint8_t Image::floatToLinearByteColor(float value)
{
	auto clampedVal = std::clamp(value, 0.f, 1.f);
	return uint8_t(clampedVal * 255);
}