#pragma once

#include "Core.h"
#include "Sampling.h"
#include "Geometry.h"
#include "Imaging.h"
#include "Materials.h"
#include "Lights.h"
#include "Scene.h"
#include "GamesEngineeringBase.h"
#include <thread>
#include <functional>
#include <queue>
#include <mutex>
#include <vector>

const int TILE_SIZE = 32;
std::mutex tileIDMutex, draw;

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom *samplers;
	//std::thread **threads;
	int numProcs;
	int tilesNumX, tilesNumY;
	std::queue<std::pair<int, int>> tileID;

	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		//film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new GaussianFilter(2.0, 0.1));
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		//threads = new std::thread*[numProcs];
		samplers = new MTRandom[numProcs]; //tile-based

		tilesNumX = (film->width + TILE_SIZE - 1) / TILE_SIZE; // ceiling
		tilesNumY = (film->height + TILE_SIZE - 1) / TILE_SIZE;

		clear();
	}
	void clear()
	{
		film->clear();
	}
	Colour computeDirect(ShadingData shadingData, Sampler* sampler)
	{
		// Is surface is specular we cannot computing direct lighting
		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// Compute direct lighting here
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler* sampler)
	{
		// Add pathtracer code here
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour direct(Ray& r, Sampler* sampler)
	{
		// Compute direct lighting for an image sampler here
		return Colour(0.0f, 0.0f, 0.0f);
	}
	Colour albedo(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return shadingData.bsdf->evaluate(shadingData, Vec3(0, 1, 0));
		}
		return scene->background->evaluate(shadingData, r.dir);
	}
	Colour viewNormals(Ray& r)
	{
		IntersectionData intersection = scene->traverse(r);
		if (intersection.t < FLT_MAX)
		{
			ShadingData shadingData = scene->calculateShadingData(intersection, r);
			return Colour(fabsf(shadingData.sNormal.x), fabsf(shadingData.sNormal.y), fabsf(shadingData.sNormal.z));
		}
		return Colour(0.0f, 0.0f, 0.0f);
	}
	
	void renderTile(unsigned int id_x, unsigned int id_y) {
		int startX = id_x * TILE_SIZE;
		int startY = id_y * TILE_SIZE;
		int endX = min(startX + TILE_SIZE, film->width);
		int endY = min(startY + TILE_SIZE, film->height);

		for (unsigned int y = startY; y < endY; y++)
		{
			for (unsigned int x = startX; x < endX; x++)
			{
				// Pick a point in the pixel (e.g. pixel centre)
				float px = x + 0.5f;
				float py = y + 0.5f;
				Ray ray = scene->camera.generateRay(px, py);
				Colour col = viewNormals(ray);
				//Colour col = albedo(ray);
				//Colour col = direct(ray, samplers);
				{
					//std::lock_guard<std::mutex> lock(draw);
					film->splat(px, py, col);
					unsigned char r = (unsigned char)(col.r * 255);
					unsigned char g = (unsigned char)(col.g * 255);
					unsigned char b = (unsigned char)(col.b * 255);
					film->tonemap(x, y, r, g, b);
					canvas->draw(x, y, r, g, b);
				}
				
			}
		}
	}
	
	void getTileID() {
		while (true) {
			unsigned int x, y;
			{
				// protect queue
				std::lock_guard<std::mutex> lock(tileIDMutex);
				if (tileID.empty()) break;
				x = tileID.front().first;
				y = tileID.front().second;
				tileID.pop();
			}
			renderTile(x, y);
		}
	}
	void render()
	{
		film->incrementSPP(); //?

		// init tile ID queue (0,0) (1,0) ... (X-1, Y-1)
		for (unsigned int y = 0; y < tilesNumY; y++)
			for (unsigned int x = 0; x < tilesNumX; x++) {
				tileID.push({ x, y });
			}

		std::vector<std::thread> threads;

		for (unsigned int i = 0; i < numProcs; i++) {
			threads.emplace_back(&RayTracer::getTileID, this);
		}
		for (auto& t : threads) {
			t.join();
		}

		//for (unsigned int y = 0; y < film->height; y++)
		//{
		//	for (unsigned int x = 0; x < film->width; x++)
		//	{
		//		float px = x + 0.5f;
		//		float py = y + 0.5f;
		//		Ray ray = scene->camera.generateRay(px, py);
		//		Colour col = viewNormals(ray);
		//		//Colour col = albedo(ray);
		//		film->splat(px, py, col);
		//		unsigned char r = (unsigned char)(col.r * 255);
		//		unsigned char g = (unsigned char)(col.g * 255);
		//		unsigned char b = (unsigned char)(col.b * 255);
		//		canvas->draw(x, y, r, g, b);
		//	}
		//}
	}
	int getSPP()
	{
		return film->SPP;
	}
	void saveHDR(std::string filename)
	{
		film->save(filename);
	}
	void savePNG(std::string filename)
	{
		stbi_write_png(filename.c_str(), canvas->getWidth(), canvas->getHeight(), 3, canvas->getBackBuffer(), canvas->getWidth() * 3);
	}
};