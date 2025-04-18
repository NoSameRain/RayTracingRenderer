﻿#pragma once

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
#include <OpenImageDenoise/oidn.hpp>

const int TILE_SIZE = 32;
std::mutex tileIDMutex, varianceMutex;
const int MAX_DEPTH = 4; 
const int MAX_SAMPLES = 10240;
const int MIN_SAMPLES = 1;
const int INIT_SAMPLES = 2;
const int MAX_VPL = 50; // max number of VPLs per pixel
struct VPL 
{
	ShadingData shadingData;
	Colour Le;
};

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
	std::vector<VPL> vpls;

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

	void presentFilmToCanvas() {
		for (unsigned int y = 0; y < film->height; y++) {
			for (unsigned int x = 0; x < film->width; x++) {
				Colour col = film->film[y * film->width + x];
				unsigned char r = (unsigned char)(col.r * 255);
				unsigned char g = (unsigned char)(col.g * 255);
				unsigned char b = (unsigned char)(col.b * 255);
				film->tonemap(x, y, r, g, b);      
				canvas->draw(x, y, r, g, b);       
			}
		}
	}
	// ----------------------------------------instant radiosity ----------------------------------------------
	void renderBlockinstantRadiosity(int start, int end, Sampler& sampler) {
		for (unsigned int y = start; y < end; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				float px = x + 0.5f;
				float py = y + 0.5f;
				Ray r = scene->camera.generateRay(px, py);
				int bounce = 0;
				// find first non-specular vertex?
				IntersectionData intersection = scene->traverse(r);
				ShadingData shadingData = scene->calculateShadingData(intersection, r);
				// compute direct lighting from each VPL
				if (shadingData.t < FLT_MAX) {
					Colour col = computeVPLsContribution(shadingData); 
					film->splat(x, y, col);
				}
			}
		}
	}
	void instantRadiosity() {
		vpls.clear();
		// -------first pass---------
		// trace paths from the light and store VPLs at each interaction
		traceVPLs(samplers[0], MAX_VPL);
		
		// -------second pass--------
		// iterate all blocks (mult-threads) and trace a path from cam to first non-specular vertex
		std::vector<std::thread> threads_ir;
		int height = film->height;
		int blockSize = height / numProcs;
		for (int t = 0; t < numProcs; t++) {
			int yStart = t * blockSize;
			int yEnd = (t == numProcs - 1) ? height : yStart + blockSize;
			threads_ir.emplace_back(&RayTracer::renderBlockinstantRadiosity, this, yStart, yEnd, std::ref(samplers[t]));
		}

		for (auto& th : threads_ir) {
			th.join();
		}
		// ---------draw------------
		presentFilmToCanvas();
	}
	// ditrct
	Colour computeVPLsContribution(ShadingData shadingData) {
		if (shadingData.bsdf->isLight() || shadingData.bsdf->isPureSpecular()) {
			return Colour(0.0f, 0.0f, 0.0f);
		}
		
		Colour col_sum(0.0f, 0.0f, 0.0f);
		for (auto& vpl : vpls) {
			// direction from intersection to vpl
			Vec3 direction = vpl.shadingData.x - shadingData.x; 
			float dist2 = direction.lengthSq();
			if (dist2 < 1e-4f) {
				continue;
			}
			direction = direction.normalize();
			// cos term of vpl and shading point
			float cos_theta_vpl = Dot(vpl.shadingData.sNormal, -direction);
			float cos_theta_x = Dot(shadingData.sNormal, direction);
			// G term
			if (cos_theta_vpl <= 0.0f || cos_theta_x <= 0.0f) continue;
			float G = (cos_theta_vpl * cos_theta_x) / dist2;
			// Visible
			if (!scene->visible(shadingData.x, vpl.shadingData.x))
				continue;

			// color * bsdf * G term
			Colour bsdf;
			bsdf = shadingData.bsdf->evaluate(shadingData, direction);
			Colour col = vpl.Le * bsdf * G;
			col_sum = col_sum + col;
		}
		return (col_sum);
	}
	void traceVPLs(Sampler& sampler, int N_VPLs) {
		for (int i = 0; i < N_VPLs; i++) {
			float pmf;
			Light* light = scene->sampleLight(sampler, pmf);
			if (light->isArea()) {
				// Sample a point on the light
				float pdfPosition, pdfDirection;
				Vec3 p = light->samplePositionFromLight(sampler, pdfPosition);
				Vec3 wi = light->sampleDirectionFromLight(sampler, pdfDirection);
				// store VPL on light source
				VPL vpl;
				ShadingData tmp;
				vpl.shadingData = ShadingData(p, light->normal(tmp, p));
				vpl.Le = light->evaluate(-wi) / (pmf * pdfPosition * (float)N_VPLs); 
				vpls.push_back(vpl);
				// emitted
				Colour Le = light->evaluate(-wi) * Dot(wi, light->normal(tmp, p)) / (pmf * pdfPosition  * (float)N_VPLs); // ????????
				// generate ray from light sample
				Ray r = Ray(p, wi);
				Colour pathThroughput(1.0f, 1.0f, 1.0f);
				// trace VPL path
				VPLTracePath(r, pathThroughput, Le, sampler);
			}
		}
	}
	void VPLTracePath(Ray& r, Colour pathThroughput, Colour Le, Sampler& sampler) {
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		// only store VPLs for non-specular surfaces
		if (shadingData.t < FLT_MAX) {
			if (!shadingData.bsdf->isLight() && !shadingData.bsdf->isPureSpecular())
			{
				// -------------store-------------------
				VPL vpl;
				vpl.shadingData = shadingData;
				//vpl.Le = pathThroughput * Le;
				vpl.Le = pathThroughput * Le * shadingData.bsdf->evaluate(shadingData, -r.dir) * fabsf(Dot(-r.dir, shadingData.sNormal));
				vpls.push_back(vpl);
			}
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
			Colour bsdfVal;
			float pdf;
			Vec3 wi = shadingData.bsdf->sample(shadingData, sampler, bsdfVal, pdf);

			pathThroughput = pathThroughput * bsdfVal * fabsf(Dot(wi, shadingData.sNormal)) / pdf;
			// spawn the next ray in the path
			r.init(shadingData.x + (wi * EPSILON), wi);
			// recursively trace the next bounce
			VPLTracePath(r, pathThroughput, Le, sampler);
		}
		return;
	}
	
	// ----------------------------------------light tracing ----------------------------------------------
	void lightTracer() {
		// trace
		for (unsigned int y = 0; y < film->height; y++)
		{
			for (unsigned int x = 0; x < film->width; x++)
			{
				lightTrace_init(samplers[0]);
			}
		}
		//denoise();
		// draw
		presentFilmToCanvas();
	}
	void connectToCamera(Vec3 p, Vec3 n, Colour col)
	{
		float x, y;
		// attempt to project the point p onto the camera plane as (x,y)
		if (scene->camera.projectOntoCamera(p, x, y)) {
			float A_film = scene->camera.Afilm;
			// compute direction from "point to be projected" to camera
			Vec3 direction = scene->camera.origin - p; 
			float dist2 = direction.lengthSq();
			direction = direction.normalize();
			// compute cosine for point side and camera side
			float cos_theta_shading = Dot(n, direction);
			float cos_theta_cam = Dot(scene->camera.viewDirection, -direction);
			// G term
			if (cos_theta_shading < 0.0f || cos_theta_cam < 0.0f) return;
			float G = (cos_theta_shading * cos_theta_cam) / dist2;
			// visible check
			if (!scene->visible(p, scene->camera.origin))
				return;
			// importance term
			float W_e = 1 / (A_film * SQ(SQ(cos_theta_cam)));
			// color * importance term * G term
			Colour color = col * W_e * G;
			film->splat(x, y, color);
		}
	}
	void lightTrace_init(Sampler& sampler)
	{
		// Sample a light
		float pmf;
		Light* light = scene->sampleLight(sampler, pmf);
		if (light->isArea())
		{
			// Sample a point on the light
			float pdfPosition, pdfDirection;
			Vec3 p = light->samplePositionFromLight(sampler, pdfPosition);
			// sample a direction from the light
			Vec3 wi = light->sampleDirectionFromLight(sampler, pdfDirection);	
			
			ShadingData tmp;
			Vec3 lightNormal = light->normal(tmp, wi);
			float cosTheta = Dot(lightNormal, wi);
			// evaluate the radiance from light sample
			Colour Le = light->evaluate(-wi) * cosTheta/ (pmf * pdfDirection * pdfPosition);
			// connect light sample directly to the camera
			connectToCamera(p, lightNormal, Le);
			// spawn a ray from the light sample
			Ray r = Ray(p, wi);
			Colour pathThroughput(1.0f, 1.0f, 1.0f);
			// recursive light tracing
			lightTracePath(r, pathThroughput, Le, sampler);
		}
	}
	void lightTracePath(Ray& r, Colour pathThroughput, Colour Le, Sampler& sampler)
	{
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData = scene->calculateShadingData(intersection, r);
		// if ray hits  
		if (shadingData.t < FLT_MAX ) {
			if (shadingData.bsdf->isLight() || shadingData.bsdf->isPureSpecular())
			{
				return;
			}
			//direction from intersection to camera
			Vec3 wi = scene->camera.origin - shadingData.x;
			wi = wi.normalize();
			// the contribution for connecting the intersection to the camera
			Colour col = pathThroughput * shadingData.bsdf->evaluate(shadingData, wi) * Le;
			// connect the intersection to the camera
			connectToCamera(shadingData.x, shadingData.sNormal, col); 
			// to limit path length
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
			// update the path throughput
			pathThroughput = pathThroughput * indirect * fabsf(Dot(wi2, shadingData.sNormal)) / pdf;
			// spawn a ray from the intersection
			r.init(shadingData.x + (wi2 * EPSILON), wi2);
			// recursively trace next bounce
			lightTracePath(r, pathThroughput, Le, sampler);
		}
		return;
	}
	// ----------------------------------------path tracing ----------------------------------------------
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

			//pathThroughput = pathThroughput * indirect * fabsf(Dot(wi, shadingData.sNormal)) / pdf;
			if (shadingData.bsdf->isPureSpecular()) {
				// Glass or mirror, already weighted in sample()
				pathThroughput = pathThroughput * indirect / pdf;
			}
			else {
				pathThroughput = pathThroughput * indirect * fabsf(Dot(wi, shadingData.sNormal)) / pdf;
			}


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
	float balanceHeuristic(float pdfA, float pdfB) {
		return pdfA / (pdfA + pdfB);
	}
	float convertPDFAreaToSolidAngle(float pdfArea, float dist2, float costheta) {
		// Convert area PDF to solid angle PDF
		if (costheta > 0.0f)
		{
			return pdfArea * dist2 / costheta;
		}
		else
		{
			return 0.0f;
		}

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
	Colour computeDirectMIS(ShadingData shadingData, Sampler& sampler)
	{
		// Is surface is specular we cannot computing direct lighting

		if (shadingData.bsdf->isPureSpecular() == true)
		{
			return Colour(0.0f, 0.0f, 0.0f);
		}
		Colour result(0.0f, 0.0f, 0.0f);
		// -------------------------------------Sample a light--------------------------------------------
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
			float cos_surface = max(Dot(wi, shadingData.sNormal), 0.0f);
			float cos_light = max(-Dot(wi, light->normal(shadingData, wi)), 0.0f);
			float GTerm = cos_surface * cos_light / l;
			if (GTerm > 0)
			{
				// Trace
				if (scene->visible(shadingData.x, p))
				{
					// sample BSDF in the same direction
					float pdfBSDF = shadingData.bsdf->PDF(shadingData, wi);
					float pdfLightSolid = convertPDFAreaToSolidAngle(pdf*pmf, l, cos_light);
					// MIS
					float weight = balanceHeuristic(pdfLightSolid, pdfBSDF);
					// Shade
					result = result + shadingData.bsdf->evaluate(shadingData, wi) * emitted * GTerm * weight / (pmf * pdf);
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
		// -------------------------------------Sample BSDF--------------------------------------------
		Colour val_bsdf;
		float pdf_bsdf;
		Vec3 wi_bsdf = shadingData.bsdf->sample(shadingData, sampler, val_bsdf, pdf_bsdf);

		Ray r = Ray(shadingData.x + (wi_bsdf*EPSILON), wi_bsdf); // generate a ray from the intersection point with direction wi_bsdf
		IntersectionData intersection = scene->traverse(r);
		ShadingData shadingData_hitLight = scene->calculateShadingData(intersection, r);
		if (shadingData_hitLight.t < FLT_MAX)
		{
			if (shadingData_hitLight.bsdf->isLight())  // if the hit surface is a light
			{
				Colour emitted2 = shadingData_hitLight.bsdf->emit(shadingData_hitLight, -wi_bsdf);
				Vec3 hitLightPoint = shadingData_hitLight.x;
				Vec3 wi = hitLightPoint - shadingData.x; // from shading point to hit point
				float dist2 = wi.lengthSq();
				wi = wi.normalize();
				float cos_light = max(0.0f, Dot(-wi, shadingData_hitLight.sNormal)); // from shading point to hit point
				float pdf_lightSolid = convertPDFAreaToSolidAngle(pdf * pmf, dist2, cos_light);;
				float weight = balanceHeuristic(pdf_bsdf, pdf_lightSolid);

				result = result + val_bsdf * emitted2 * max(0.0f, Dot(wi_bsdf, shadingData.sNormal)) * weight / pdf_bsdf;
			}
		}

		return result;

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
	
	void adaptiveSampling(unsigned int id_x, unsigned int id_y, unsigned int tileIndex, unsigned int id_thread) {
		int startX = id_x * TILE_SIZE;
		int startY = id_y * TILE_SIZE;
		int endX = min(startX + TILE_SIZE, film->width);
		int endY = min(startY + TILE_SIZE, film->height);
		int blockSize = (endX - startX) * (endY - startY);

		// adaptive sampling
		std::vector<Colour> blockEstimatedValues(blockSize, Colour(0.0f,0.0f,0.0f));

		// 1 render img at low spp - calculate estimated values for each pixel in a block
		int pixelIndex = 0;
		for (unsigned int y = startY; y < endY; y++)
		{
			for (unsigned int x = startX; x < endX; x++)
			{
				Colour estimatedValue;
				Colour sum_init(0.0f, 0.0f, 0.0f);

				// initial sampling
				for (unsigned int i = 0; i < INIT_SAMPLES; i++) {
					// Pick a point in the pixel (e.g. pixel centre)
					float px = x + 0.5f;
					float py = y + 0.5f;
					Ray ray = scene->camera.generateRay(px, py);
					Colour pathThroughput(1.0f, 1.0f, 1.0f);
					//std::cout << "thread: " << id_thread  << std::endl;
					Colour col = pathTrace(ray, pathThroughput, 0, samplers[id_thread]);

					sum_init = sum_init + col;
				}
				estimatedValue = sum_init / (float)INIT_SAMPLES;
				//blockEstimatedValues.push_back(estimatedValue);
				blockEstimatedValues[pixelIndex] = estimatedValue;
				pixelIndex++;
			}
		}
		// 2 calculate ground truth using mean over region
		Colour estimated_sum(0.0f, 0.0f, 0.0f);
		for (int i = 0; i < blockEstimatedValues.size(); i++) {
			estimated_sum = estimated_sum + blockEstimatedValues[i];
		}
		Colour groundTruth = estimated_sum / (float)pixelIndex;

		// 3 calculate variance per block
		Colour sum(0.0f, 0.0f, 0.0f);
		for (int i = 0; i < blockEstimatedValues.size(); i++) {
			Colour tmp = blockEstimatedValues[i] - groundTruth;
			sum = sum + tmp * tmp;
		}
		float variance = ((sum.r + sum.g + sum.b) / 3.0f) / (float)(pixelIndex - 1);

		// 4 store var per block
		{
			std::lock_guard<std::mutex> lock(varianceMutex);
			tileVariances[tileIndex] = variance;
		}

	}
	
	void sampleTileWithWeight(unsigned int id_x, unsigned int id_y, unsigned int tileIndex, unsigned int id_thread) {
		int startX = id_x * TILE_SIZE;
		int startY = id_y * TILE_SIZE;
		int endX = min(startX + TILE_SIZE, film->width);
		int endY = min(startY + TILE_SIZE, film->height);
		
		float weight = tileWeights[tileIndex];
		weight = sqrt(weight);
		//weight = weight / (weight + 1.0f);  // make higher variance more stable
		int sample = (int)(weight * MAX_SAMPLES);
		sample = max(sample, MIN_SAMPLES);

		std::cout << "sample: " << sample << std::endl;
		//std::cout << "weight: " << weight << std::endl;

		for (unsigned int y = startY; y < endY; y++)
		{
			for (unsigned int x = startX; x < endX; x++)
			{
				// Pick a point in the pixel (e.g. pixel centre)
				float px = x + 0.5f;
				float py = y + 0.5f;
				Colour col(0.0f,0.0f,0.0f);

				for (unsigned int i = 0; i < sample; i++) {
					
					Ray ray = scene->camera.generateRay(px, py);
					Colour pathThroughput(1.0f, 1.0f, 1.0f);
					col = col + pathTrace(ray, pathThroughput, 0, samplers[id_thread]);
				}
				col = col / (float)sample;
				film->splat(px, py, col);
			}
		}
	}
	
	void adaptiveRender() {
		// init tile ID queue (0,0) (1,0) ... (X-1, Y-1)
		for (unsigned int y = 0; y < tilesNumY; y++)
			for (unsigned int x = 0; x < tilesNumX; x++) {
				tileID.push({ x, y });
			}
		// --------------SAMPLE-------------------
		std::vector<std::thread> threads_sample;

		for (unsigned int i = 0; i < numProcs; i++) {
			threads_sample.emplace_back([this, i]() {
				while (true) { // get tile id
					unsigned int x, y;
					{
						std::lock_guard<std::mutex> lock(tileIDMutex);
						if (tileID.empty()) break;
						x = tileID.front().first;
						y = tileID.front().second;
						tileID.pop();
					}
					// calculate index of current tile
					unsigned int tileIndex = y * tilesNumX + x;
					adaptiveSampling(x, y, tileIndex, i);
				}
				});
		}
		for (auto& t : threads_sample) {
			t.join();
		}

		// calculate total variance for all tiles
		float totalVariance = 0.0f;
		for (float v : tileVariances)
			totalVariance += v;

		// calculate weights for each tile
		for (unsigned int i = 0; i < totalTiles; i++) {
			tileWeights[i] = (totalVariance > 0.0f) ? tileVariances[i] / totalVariance : 0.0f;
			// instead of tileVariances[i] / total, use sqrt or log for stability
			//tileWeights[i] = sqrt(tileVariances[i] + EPSILON); 
		}

		// -------------RENDER---------------------------------------------------
		for (unsigned int y = 0; y < tilesNumY; y++)
			for (unsigned int x = 0; x < tilesNumX; x++) {
				tileID.push({ x, y });
			}
		std::vector<std::thread> threads_render;
		for (unsigned int i = 0; i < numProcs; i++) {
			//threads_render.emplace_back(&RayTracer::getTileID, this);
			threads_render.emplace_back([this, i]() {
				while (true) { // get tile id
					unsigned int x, y;
					{
						std::lock_guard<std::mutex> lock(tileIDMutex);
						if (tileID.empty()) break;
						x = tileID.front().first;
						y = tileID.front().second;
						tileID.pop();
					}
					// calculate index of current tile
					unsigned int tileIndex = y * tilesNumX + x;
					sampleTileWithWeight(x, y, tileIndex, i);
				}
				});
		}
		for (auto& t : threads_render) {
			t.join();
		}
		presentFilmToCanvas();
	}


	void denoise() { // only used beauty image to denoise here, no AOVs
		int width = film->width;
		int height = film->height;
		int numPixels = width * height;
		size_t bufferSize = numPixels * 3 * sizeof(float);

		// create device
		oidn::DeviceRef device = oidn::newDevice();
		device.commit();

		// buffer to input and output image data
		oidn::BufferRef colorBuf = device.newBuffer(bufferSize);

		// create a filter for denoising a color image
		oidn::FilterRef filter = device.newFilter("RT");

		filter.setImage("color", colorBuf, oidn::Format::Float3, width, height);
		filter.setImage("output", colorBuf, oidn::Format::Float3, width, height);

		filter.set("hdr", true);
		filter.commit();

		// fill the input image buffers copy from film
		float* colorPtr = (float*)colorBuf.getData();
		for (int i = 0; i < numPixels; i++) {
			colorPtr[3 * i + 0] = film->film[i].r;
			colorPtr[3 * i + 1] = film->film[i].g;
			colorPtr[3 * i + 2] = film->film[i].b;
		}

		// filter the image
		filter.execute();

		// re-write to film
		colorPtr = (float*)colorBuf.getData();
		for (int i = 0; i < numPixels; i++) {
			film->film[i].r = colorPtr[3 * i + 0];
			film->film[i].g = colorPtr[3 * i + 1];
			film->film[i].b = colorPtr[3 * i + 2];
		}
	
	}

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
				Colour col = pathTrace(ray, pathThroughput, 0, sample); // sampler per thread
				film->splat(px, py, col);

			}
		}
	}

	void getTileID(int thread_id) {
		while (true) { // pop IDs until queue is empty
			unsigned int x, y;
			{
				// protect shared ID queue
				std::lock_guard<std::mutex> lock(tileIDMutex);
				if (tileID.empty()) break;
				x = tileID.front().first;
				y = tileID.front().second;
				tileID.pop(); 
			}
			// get tile ID and sampler ID, then render tile
			renderTile(x, y, samplers[thread_id]);
		}
	}
	
	void pathTracerTileBased() {
		// init tile ID queue (0,0) (1,0) ... (X-1, Y-1) with all tiles
		for (unsigned int y = 0; y < tilesNumY; y++)
			for (unsigned int x = 0; x < tilesNumX; x++) {
				tileID.push({ x, y });
			}
		std::vector<std::thread> threads_tile_render;
		// create threads to render each tile
		for (unsigned int i = 0; i < numProcs; i++) {
			threads_tile_render.emplace_back(&RayTracer::getTileID, this, i);
		}
		for (auto& t : threads_tile_render) {
			t.join();
		}
		//denoise();
		// draw to canvas
		presentFilmToCanvas();
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

	void render()
	{
		film->incrementSPP(); 

		//adaptiveRender();
		pathTracerTileBased();
		//basicRender();
		//lightTracer();
		//instantRadiosity();
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