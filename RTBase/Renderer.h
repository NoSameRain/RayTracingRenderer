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
std::mutex tileIDMutex, varianceMutex;
const int MAX_DEPTH = 4; 
const int MAX_SAMPLES = 100000;
const int MIN_SAMPLES = 1;
const int INIT_SAMPLES = 4;

class RayTracer
{
public:
	Scene* scene;
	GamesEngineeringBase::Window* canvas;
	Film* film;
	MTRandom *samplers;
	//std::thread **threads;
	int numProcs;
	int tilesNumX, tilesNumY,totalTiles;
	std::queue<std::pair<int, int>> tileID;
	std::vector<float> tileVariances, tileWeights;

	void init(Scene* _scene, GamesEngineeringBase::Window* _canvas)
	{
		scene = _scene;
		canvas = _canvas;
		film = new Film();
		film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new BoxFilter());
		//film->init((unsigned int)scene->camera.width, (unsigned int)scene->camera.height, new GaussianFilter(2.0, 0.1));
		SYSTEM_INFO sysInfo;
		GetSystemInfo(&sysInfo);
		numProcs = sysInfo.dwNumberOfProcessors;
		samplers = new MTRandom[numProcs]; 

		tilesNumX = (film->width + TILE_SIZE - 1) / TILE_SIZE; // ceiling
		tilesNumY = (film->height + TILE_SIZE - 1) / TILE_SIZE;
		totalTiles = tilesNumX * tilesNumY;
		tileVariances = std::vector<float>(totalTiles, 0.0f);
		tileWeights = std::vector<float>(totalTiles, 0.0f);

		clear();
	}
	void clear()
	{
		film->clear();
	}
	Colour computeDirect(ShadingData shadingData, Sampler& sampler)
	{
		// Is surface is specular we cannot computing direct lighting

		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		// Sample a light
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);
		// Sample a point on the light
		float pdf;
		Colour emitted;

		Vec3 p = light->sample(shadingData, sampler, emitted, pdf);
		if (light->isArea())
		{
			// Calculate GTerm
			Vec3 wi = p - shadingData.x;
			float l = wi.lengthSq();
			wi = wi.normalize();
			float GTerm = (max(Dot(wi, shadingData.sNormal), 0.0f) * max(-Dot(wi, light->normal(shadingData, wi)), 0.0f)) / l;
			if (GTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x, p))
				{
					// Shade
					return shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm / (pmf * pdf);
				}
			}
		}
		else
		{
			// Calculate GTerm
			Vec3 wi = p;
			float GTerm = max(Dot(wi, shadingData.sNormal), 0.0f);
			if (GTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x, shadingData.x + (p * 10000.0f)))
				{
					// Shade DirectIllum=BSDF × Emission × G × Visibility.
					return shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm / (pmf * pdf);
				}
			}
		}
		return Colour(0.0f, 0.0f, 0.0f);

	}
	void connectToCamera(Vec3 p, Vec3 n, Colour col)
	{
		float x, y;
		if (scene->camera.projectOntoCamera(p, x, y) && scene->visible(p,scene->camera.origin)) {
			float A_film = scene->camera.Afilm;
			Vec3 direction = scene->camera.origin - p; // ?????
			float dist2 = direction.lengthSq();
			direction = direction.normalize();

			float cos_theta_shading = Dot(n, direction);
			float cos_theta_cam = Dot(scene->camera.viewDirection, direction);

			// G term
			float G = (max(cos_theta_shading, 0.0f) * max(-cos_theta_cam, 0.0f)) / dist2;
			//float G = max(cos_theta_shading, 0.0f) / dist2;
			// 
			// importance  term
			float W_e = 1 / (A_film * SQ(SQ(cos_theta_cam)));
			// color * importance term * G term
			Colour color = col * W_e * G;
			film->splat(x, y, color);
		}
	}

	void lightTrace(Sampler& sampler)
	{
		// Sample a light
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);
		if (light->isArea())
		{
			// Sample a point on the light
			float pdfPosition, pdfDirection;
			Vec3 p = light->samplePositionFromLight(sampler, pdfPosition);
			Vec3 wi = light->sampleDirectionFromLight(sampler, pdfDirection);	
			
			ShadingData tmp;
			Vec3 lightNormal = light->normal(tmp, wi);
			float cosTheta = Dot(lightNormal, wi);
			
			Colour col = light->evaluate(-wi) * cosTheta/ (pmf * pdfDirection * pdfPosition);
			Colour Le = col * cosTheta;

			connectToCamera(p, lightNormal, col);

			Ray r = Ray(p, wi);
			Colour pathThroughput(1.0f, 1.0f, 1.0f);
			lightTracePath(r, pathThroughput, Le, sampler);
		}
	}

	void lightTracePath(Ray& r, Colour pathThroughput, Colour Le, Sampler& sampler)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX ) {
			// direction from intersection to camera
			Vec3 wi = scene->camera.origin - shadingData.x;
			wi = wi.normalize();
			// the contribution for connecting the intersection to the camera
			Colour col = pathThroughput * shadingData.bsdf->evaluate(shadingData, wi) * Le;

			connectToCamera(shadingData.x, shadingData.sNormal, col); //??????????????
			
			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);
			if (sampler.next() < russianRouletteProbability)
			{
				pathThroughput = pathThroughput / russianRouletteProbability;
			}
			else
			{
				return;
			}
			// sample the BSDF to get next bounce direction
			Colour indirect;
			float pdf;
			Vec3 wi2 = shadingData.bsdf->sample(shadingData, sampler, indirect, pdf);

			pathThroughput = pathThroughput * indirect * fabsf(Dot(wi2, shadingData.sNormal)) / pdf;

			r.init(shadingData.x + (wi2 * EPSILON), wi2);

			lightTracePath(r, pathThroughput, Le, sampler);
		}
		return;
	}

	Colour pathTrace(Ray& r, Colour& pathThroughput, int depth, Sampler& sampler, bool canHitLight = true)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				if (canHitLight == true) // specular
				{
					return pathThroughput * shadingData.bsdf->emit(shadingData, shadingData.wo);
				}
				else // diffuse...
				{
					return Colour(0.0f, 0.0f, 0.0f);
				}
			}
			// sample light ------------------------------------------------------------------------------------
			Colour direct = pathThroughput * computeDirect(shadingData, sampler);

			// sample BSDF ------------------------------------------------------------------------------------
			if (depth > MAX_DEPTH)
			{
				return direct;
			}
			float russianRouletteProbability = min(pathThroughput.Lum(), 0.9f);
			if (sampler.next() < russianRouletteProbability)
			{
				pathThroughput = pathThroughput / russianRouletteProbability;
			}
			else
			{
				return direct;
			}
			// replace for material
			Colour indirect;
			float pdf;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, indirect, pdf);

			pathThroughput = pathThroughput * indirect * fabsf(Dot(wi, shadingData.sNormal)) / pdf;

			/*Colour bsdf;
			float pdf;
			Vec3 wi = SamplingDistributions::cosineSampleHemisphere(sampler.next(), sampler.next());
			pdf = SamplingDistributions::cosineHemispherePDF(wi);
			wi = shadingData.frame.toWorld(wi);
			bsdf = shadingData.bsdf->evaluate(shadingData, wi);
			pathThroughput = pathThroughput * bsdf * fabsf(Dot(wi, shadingData.sNormal)) / pdf;*/
			
			r.init(shadingData.x + (wi * EPSILON), wi);


			return (direct + pathTrace(r, pathThroughput, depth + 1, sampler, shadingData.bsdf->isPureSpecular()));
		}
		return scene->background->evaluate(r.dir);

	}
	Colour direct(Ray& r, Sampler& sampler)
	{
		// Compute direct lighting for an image sampler here
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		if (shadingData.t < FLT_MAX)
		{
			if (shadingData.bsdf->isLight())
			{
				return shadingData.bsdf->emit(shadingData, shadingData.wo);
			}
			return computeDirect(shadingData, sampler);
		}
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
		return scene->background->evaluate(r.dir);
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
	
	//void adaptiveSampling(unsigned int id_x, unsigned int id_y, unsigned int tileIndex) {
	//	int startX = id_x * TILE_SIZE;
	//	int startY = id_y * TILE_SIZE;
	//	int endX = min(startX + TILE_SIZE, film->width);
	//	int endY = min(startY + TILE_SIZE, film->height);
	//	int blockSize = (endX - startX) * (endY - startY);

	//	// adaptive sampling
	//	std::vector<Colour> blockEstimatedValues(blockSize, Colour(0.0f,0.0f,0.0f));

	//	// 1 render img at low spp - calculate estimated values for each pixel in a block
	//	int pixelIndex = 0;
	//	for (unsigned int y = startY; y < endY; y++)
	//	{
	//		for (unsigned int x = startX; x < endX; x++)
	//		{
	//			Colour estimatedValue;
	//			Colour sum_init(0.0f, 0.0f, 0.0f);

	//			// initial sampling
	//			for (unsigned int i = 0; i < INIT_SAMPLES; i++) {
	//				// Pick a point in the pixel (e.g. pixel centre)
	//				float px = x + 0.5f;
	//				float py = y + 0.5f;
	//				Ray ray = scene->camera.generateRay(px, py);
	//				Colour pathThroughput(1.0f, 1.0f, 1.0f);
	//				Colour col = pathTrace(ray, pathThroughput, 0, samplers);

	//				sum_init = sum_init + col;
	//			}
	//			estimatedValue = sum_init / (float)INIT_SAMPLES;
	//			//blockEstimatedValues.push_back(estimatedValue);
	//			blockEstimatedValues[pixelIndex] = estimatedValue;
	//			pixelIndex++;
	//		}
	//	}
	//	// 2 calculate ground truth using mean over region
	//	Colour estimated_sum(0.0f, 0.0f, 0.0f);
	//	for (int i = 0; i < blockEstimatedValues.size(); i++) {
	//		estimated_sum = estimated_sum + blockEstimatedValues[i];
	//	}
	//	Colour groundTruth = estimated_sum / (float)pixelIndex;

	//	// 3 calculate variance per block
	//	Colour sum(0.0f, 0.0f, 0.0f);
	//	for (int i = 0; i < blockEstimatedValues.size(); i++) {
	//		Colour tmp = blockEstimatedValues[i] - groundTruth;
	//		sum = sum + tmp * tmp;
	//	}
	//	float variance = ((sum.r + sum.g + sum.b) / 3.0f) / (float)(pixelIndex - 1);

	//	// 4 store var per block
	//	{
	//		std::lock_guard<std::mutex> lock(varianceMutex);
	//		tileVariances[tileIndex] = variance;
	//	}

	//}
	//
	//void renderTileWithSample(unsigned int id_x, unsigned int id_y, unsigned int tileIndex) {
	//	int startX = id_x * TILE_SIZE;
	//	int startY = id_y * TILE_SIZE;
	//	int endX = min(startX + TILE_SIZE, film->width);
	//	int endY = min(startY + TILE_SIZE, film->height);
	//	int sample = (int)(tileWeights[tileIndex] * MAX_SAMPLES);
	//	if (sample < MIN_SAMPLES) sample = MIN_SAMPLES;

	//	//std::cout << "sample: " << sample << std::endl;

	//	for (unsigned int y = startY; y < endY; y++)
	//	{
	//		for (unsigned int x = startX; x < endX; x++)
	//		{
	//			// Pick a point in the pixel (e.g. pixel centre)
	//			float px = x + 0.5f;
	//			float py = y + 0.5f;
	//			Colour col(0.0f,0.0f,0.0f);

	//			for (unsigned int i = 0; i < sample; i++) {
	//				
	//				Ray ray = scene->camera.generateRay(px, py);
	//				//Colour col = viewNormals(ray);
	//				//Colour col = albedo(ray);
	//				//Colour col = direct(ray, samplers);
	//				Colour pathThroughput(1.0f, 1.0f, 1.0f);
	//				col = col + pathTrace(ray, pathThroughput, 0, samplers);
	//			}
	//			col = col / (float)sample;
	//			film->splat(px, py, col);
	//			unsigned char r = (unsigned char)(col.r * 255);
	//			unsigned char g = (unsigned char)(col.g * 255);
	//			unsigned char b = (unsigned char)(col.b * 255);
	//			film->tonemap(x, y, r, g, b);
	//			canvas->draw(x, y, r, g, b);
	//		}
	//	}
	//}
	//
	//void adaptiveRender() {
	//	// init tile ID queue (0,0) (1,0) ... (X-1, Y-1)
	//	for (unsigned int y = 0; y < tilesNumY; y++)
	//		for (unsigned int x = 0; x < tilesNumX; x++) {
	//			tileID.push({ x, y });
	//		}
	//	// --------------SAMPLE---------------------------------------------------------
	//	std::vector<std::thread> threads_sample;

	//	for (unsigned int i = 0; i < numProcs; i++) {
	//		threads_sample.emplace_back([&]() {
	//			while (true) { // get tile id
	//				unsigned int x, y;
	//				{
	//					std::lock_guard<std::mutex> lock(tileIDMutex);
	//					if (tileID.empty()) break;
	//					x = tileID.front().first;
	//					y = tileID.front().second;
	//					tileID.pop();
	//				}
	//				// calculate index of current tile
	//				unsigned int tileIndex = y * tilesNumX + x;
	//				adaptiveSampling(x, y, tileIndex);
	//			}
	//			});
	//	}
	//	for (auto& t : threads_sample) {
	//		t.join();
	//	}

	//	// calculate total variance for all tiles
	//	float totalVariance = 0.0f;
	//	for (float v : tileVariances)
	//		totalVariance += v;

	//	// calculate weights for each tile
	//	for (unsigned int i = 0; i < totalTiles; i++) {
	//		tileWeights[i] = (totalVariance > 0.0f) ? tileVariances[i] / totalVariance : 0.0f;
	//	}

	//	// -------------RENDER---------------------------------------------------
	//	for (unsigned int y = 0; y < tilesNumY; y++)
	//		for (unsigned int x = 0; x < tilesNumX; x++) {
	//			tileID.push({ x, y });
	//		}
	//	std::vector<std::thread> threads_render;
	//	for (unsigned int i = 0; i < numProcs; i++) {
	//		//threads_render.emplace_back(&RayTracer::getTileID, this);
	//		threads_render.emplace_back([&]() {
	//			while (true) { // get tile id
	//				unsigned int x, y;
	//				{
	//					std::lock_guard<std::mutex> lock(tileIDMutex);
	//					if (tileID.empty()) break;
	//					x = tileID.front().first;
	//					y = tileID.front().second;
	//					tileID.pop();
	//				}
	//				// calculate index of current tile
	//				unsigned int tileIndex = y * tilesNumX + x;
	//				renderTileWithSample(x, y, tileIndex);
	//			}
	//			});
	//	}
	//	for (auto& t : threads_render) {
	//		t.join();
	//	}
	//}
	
	void renderTile(unsigned int id_x, unsigned int id_y, Sampler& sample) {
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
				//Colour col = viewNormals(ray);
				//Colour col = albedo(ray);
				//Colour col = direct(ray, samplers);
				Colour pathThroughput(1.0f, 1.0f, 1.0f);
				//Colour col = pathTrace(ray, pathThroughput, 0, *samplers);
				Colour col = pathTrace(ray, pathThroughput, 0, sample); // sampler per thread
				film->splat(px, py, col);
				unsigned char r = (unsigned char)(col.r * 255);
				unsigned char g = (unsigned char)(col.g * 255);
				unsigned char b = (unsigned char)(col.b * 255);
				film->tonemap(x, y, r, g, b);
				canvas->draw(x, y, r, g, b);

			}
		}
	}

	void getTileID(int thread_id) {
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
			renderTile(x, y, samplers[thread_id]);
		}
	}
	
	void tileBasedRender() {
		for (unsigned int y = 0; y < tilesNumY; y++)
			for (unsigned int x = 0; x < tilesNumX; x++) {
				tileID.push({ x, y });
			}

		std::vector<std::thread> threads_tile_render;

		for (unsigned int i = 0; i < numProcs; i++) {
			threads_tile_render.emplace_back(&RayTracer::getTileID, this, i);
		}
		for (auto& t : threads_tile_render) {
			t.join();
		}
	}

	void basicRender() {
		for (unsigned int y = 0; y < film->height; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				float px = x + 0.5f;
				float py = y + 0.5f;
				Ray ray = scene->camera.generateRay(px, py);
				//Colour col = viewNormals(ray);
				//Colour col = albedo(ray);
				Colour pathThroughput(1.0f, 1.0f, 1.0f);
				Colour col = pathTrace(ray, pathThroughput, 0, *samplers); // sampler per thread
				film->splat(px, py, col);
				unsigned char r = (unsigned char)(col.r * 255);
				unsigned char g = (unsigned char)(col.g * 255);
				unsigned char b = (unsigned char)(col.b * 255);
				canvas->draw(x, y, r, g, b);
			}
		}
	}

	void lightTracingRender() {
		for (unsigned int y = 0; y < film->height; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				lightTrace(samplers[0]);
			}
		}
		
		for (unsigned int y = 0; y < film->height; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				int id = (y * film->width) + x;
				Colour col = film->film[id];
				unsigned char r = (unsigned char)(col.r * 255);
				unsigned char g = (unsigned char)(col.g * 255);
				unsigned char b = (unsigned char)(col.b * 255);
				film->tonemap(x, y, r, g, b);
				
				canvas->draw(x, y, r, g, b);
			}
		}
	}

	void render()
	{
		film->incrementSPP(); 
		//adaptiveRender();
		//tileBasedRender();
		//basicRender();
		lightTracingRender();
		
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