#include "dsf/mobility/TrafficLight.hpp"
#include "dsf/geometry/Point.hpp"

#include <benchmark/benchmark.h>
#include <memory>

static void BM_TrafficLight_Construction(benchmark::State& state) {
  for (auto _ : state) {
    dsf::mobility::TrafficLight tl(0, 60);
    benchmark::DoNotOptimize(tl);
  }
}

static void BM_TrafficLight_ConstructionWithPoint(benchmark::State& state) {
  dsf::geometry::Point point{0.0, 0.0};
  for (auto _ : state) {
    dsf::mobility::TrafficLight tl(0, 60, point);
    benchmark::DoNotOptimize(tl);
  }
}

static void BM_TrafficLight_OperatorIncrement(benchmark::State& state) {
  dsf::mobility::TrafficLight tl(0, 60);
  for (auto _ : state) {
    ++tl;
    benchmark::DoNotOptimize(tl);
  }
}

static void BM_TrafficLight_SetCycle(benchmark::State& state) {
  dsf::mobility::TrafficLight tl(0, 60);
  dsf::mobility::TrafficLightCycle cycle(30, 0);
  for (auto _ : state) {
    tl.setCycle(1, dsf::Direction::STRAIGHT, cycle);
  }
}

static void BM_TrafficLight_SetComplementaryCycle(benchmark::State& state) {
  for (auto _ : state) {
    dsf::mobility::TrafficLight tl(0, 60);
    dsf::mobility::TrafficLightCycle cycle(30, 0);
    tl.setCycle(1, dsf::Direction::STRAIGHT, cycle);
    tl.setComplementaryCycle(2, 1);
  }
}

static void BM_TrafficLight_IsGreen(benchmark::State& state) {
  dsf::mobility::TrafficLight tl(0, 60);
  dsf::mobility::TrafficLightCycle cycle(30, 0);
  tl.setCycle(1, dsf::Direction::STRAIGHT, cycle);
  for (auto _ : state) {
    bool green = tl.isGreen(1, dsf::Direction::STRAIGHT);
    benchmark::DoNotOptimize(green);
  }
}

static void BM_TrafficLight_MeanGreenTime(benchmark::State& state) {
  dsf::mobility::TrafficLight tl(0, 60);
  dsf::mobility::TrafficLightCycle cycle1(30, 0);
  dsf::mobility::TrafficLightCycle cycle2(20, 30);
  tl.setCycle(1, dsf::Direction::STRAIGHT, cycle1);
  tl.setCycle(2, dsf::Direction::LEFT, cycle2);
  for (auto _ : state) {
    double mean = tl.meanGreenTime(false);
    benchmark::DoNotOptimize(mean);
  }
}

static void BM_TrafficLight_ResetCycles(benchmark::State& state) {
  dsf::mobility::TrafficLight tl(0, 60);
  dsf::mobility::TrafficLightCycle cycle(30, 0);
  tl.setCycle(1, dsf::Direction::STRAIGHT, cycle);
  for (auto _ : state) {
    tl.resetCycles();
  }
}

static void BM_TrafficLight_IncreasePhases(benchmark::State& state) {
  dsf::mobility::TrafficLight tl(0, 60);
  dsf::mobility::TrafficLightCycle cycle(30, 0);
  tl.setCycle(1, dsf::Direction::STRAIGHT, cycle);
  for (auto _ : state) {
    tl.increasePhases(5);
  }
}

BENCHMARK(BM_TrafficLight_Construction);
BENCHMARK(BM_TrafficLight_ConstructionWithPoint);
BENCHMARK(BM_TrafficLight_OperatorIncrement);
BENCHMARK(BM_TrafficLight_SetCycle);
BENCHMARK(BM_TrafficLight_SetComplementaryCycle);
BENCHMARK(BM_TrafficLight_IsGreen);
BENCHMARK(BM_TrafficLight_MeanGreenTime);
BENCHMARK(BM_TrafficLight_ResetCycles);
BENCHMARK(BM_TrafficLight_IncreasePhases);

BENCHMARK_MAIN();