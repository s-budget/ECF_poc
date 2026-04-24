#pragma once
#include <string>
#include <vector>
#include <map>
#include <array>
#include <unordered_map>
#include <fstream>
#define _USE_MATH_DEFINES
#include <math.h>
#include <stdexcept>
#include <algorithm>
#include <numeric>

// nlohmann/json
#include "json.hpp"
using json = nlohmann::json;
enum class CardinalDir { North = 0, East = 1, South = 2, West = 3 };

enum class TurnType { Straight = 0, TurnLeft = 1, TurnRight = 2 };

enum class RoadDir { Inbound, Outbound };

struct MovementKey {
    CardinalDir from;
    TurnType    turn;
    bool operator==(MovementKey const& o) const {
        return from == o.from && turn == o.turn;
    }
};

struct MovementKeyHash {
    std::size_t operator()(MovementKey const& k) const noexcept {
        return std::hash<int>{}(static_cast<int>(k.from) * 3 +
                                static_cast<int>(k.turn));
    }
};

struct RoadKey {
    CardinalDir cardinal;
    RoadDir     direction;
    bool operator==(RoadKey const& o) const {
        return cardinal == o.cardinal && direction == o.direction;
    }
};

struct RoadKeyHash {
    std::size_t operator()(RoadKey const& k) const noexcept {
        return std::hash<int>{}(static_cast<int>(k.cardinal) * 2 +
                                static_cast<int>(k.direction));
    }
};

inline std::string movementName(MovementKey const& k) {
    static const char* dirs[]  = {"North", "East", "South", "West"};
    static const char* turns[] = {"Straight", "Left", "Right"};
    return std::string(dirs[static_cast<int>(k.from)]) + "_" +
           turns[static_cast<int>(k.turn)];
}

/// Lane id format: roadId + "_" + laneIndex  e.g. "road_1_0_1_0"
using LaneList = std::vector<std::string>;

using MovementMap =
    std::unordered_map<MovementKey, LaneList, MovementKeyHash>;

/// Lookup: (CardinalDir, RoadDir) -> road id
using RoadMap =
    std::unordered_map<RoadKey, std::string, RoadKeyHash>;

/// phase index -> list of lane ids that are open during that phase
using PhaseLaneMap = std::map<int, LaneList>;

/// Lookup: road_id -> (TurnType -> lane_ids going that direction on that road)
/// Built from the movements map after Step D.
/// For a given road, e.g. roadTurnLanes["road_1_0_1"][TurnType::Straight]
/// gives all lane ids on that road whose vehicles go straight.
using RoadTurnLaneMap =
    std::unordered_map<std::string, std::map<TurnType, LaneList>>;

/// One directed movement active during a phase.
struct TurnMovement {
    std::string inboundRoadId;
    std::string outboundRoadId;
    LaneList    inboundLaneIds;   ///< sorted lane ids on the inbound road
};

/// All movements simultaneously green in one traffic-light phase.
struct PhaseData {
    std::vector<TurnMovement> movements;
};


struct IntersectionData {
    std::string id;
    double      x, y;
    bool        isVirtual;

    /// Up to 12 movement buckets (approach direction x turn type).
    /// Each bucket holds the ids of the inbound lanes for that movement.
    /// Only buckets that have at least one lane are present.
    MovementMap movements;

    /// Lookup a road by cardinal direction and inbound/outbound.
    /// e.g. roads[{CardinalDir::North, RoadDir::Inbound}]  = "road_1_0_1"
    ///      roads[{CardinalDir::North, RoadDir::Outbound}] = "road_1_1_3"
    RoadMap roads;

    /// For each traffic light phase, the sorted list of lane ids that are
    /// unlocked (green) during that phase.
    /// e.g. phaseLanes[0] = {"road_0_1_0_2", "road_0_1_0_3", ...}
    ///      phaseLanes[1] = {"road_1_0_1_0", ...}
    PhaseLaneMap phaseLanes;

    /// Lookup: road_id -> (TurnType -> list of lane_ids on that road going
    /// that direction). Derived from movements after Step D.
    /// e.g. roadTurnLanes["road_1_0_1"][TurnType::TurnLeft] = {"road_1_0_1_0"}
    ///      roadTurnLanes["road_1_0_1"][TurnType::Straight]  = {"road_1_0_1_1"}
    RoadTurnLaneMap roadTurnLanes;

    /// Per-phase breakdown: index matches phaseLanes key.
    /// Each entry lists every inbound->outbound TurnMovement green in that phase.
    std::vector<PhaseData> phasesData;
};

// ─────────────────────────────────────────────────────────────────────────────
//  Internal helpers
// ─────────────────────────────────────────────────────────────────────────────
namespace detail {
    inline json loadJson(std::string const& path) {
        std::ifstream f(path);
        if (!f.is_open())
            throw std::runtime_error("Cannot open file: " + path);
        json j;
        f >> j;
        return j;
    }

    inline TurnType parseTurnType(std::string const& s) {
        if (s == "go_straight") return TurnType::Straight;
        if (s == "turn_left")   return TurnType::TurnLeft;
        if (s == "turn_right")  return TurnType::TurnRight;
        throw std::runtime_error("Unknown road-link type: " + s);
    }

    /// Smallest angular distance between two atan2 angles, result in [0, π].
    inline double angleDiff(double a, double b) {
        double d = std::fmod(std::abs(a - b), 2.0 * M_PI);
        return (d > M_PI) ? 2.0 * M_PI - d : d;
    }

    struct CardinalInfo {
        CardinalDir dir;
        /// Ideal atan2(dy, dx) angle of a vector pointing FROM the far road
        /// endpoint TOWARD the intersection centre, for traffic coming from
        /// this cardinal direction.  JSON uses screen coords (+Y = South):
        ///   From West  → incoming vector points East  → atan2(0,+1)  =  0
        ///   From North → incoming vector points South → atan2(+1,0)  = +π/2
        ///   From East  → incoming vector points West  → atan2(0,-1)  = ±π
        ///   From South → incoming vector points North → atan2(-1,0)  = -π/2
        double incomingAngle;
    };

    inline std::array<CardinalInfo, 4> const& cardinalTable() {
        static const std::array<CardinalInfo, 4> t = {{
            { CardinalDir::West,   0.0          },
            { CardinalDir::North,  M_PI / 2.0   },
            { CardinalDir::East,   M_PI         },
            { CardinalDir::South, -M_PI / 2.0   },
        }};
        return t;
    }

    /// Assign up to 4 inbound roads to cardinal directions using rotational order.
    /// Only 4 offsets are tried instead of 4! = 24 permutations.
    /// Returns: assignment[i] = index into cardinalTable() for roads[i], or -1.
    inline std::vector<int> assignRoadsToCardinals(
        std::vector<double> const&         roadAngles,
        std::array<CardinalInfo, 4> const& cardinals
    ) {
        int nRoads = static_cast<int>(roadAngles.size());

        std::vector<int> roadOrder(nRoads);
        std::iota(roadOrder.begin(), roadOrder.end(), 0);
        std::sort(roadOrder.begin(), roadOrder.end(), [&](int a, int b) {
            return roadAngles[a] < roadAngles[b];
        });

        std::vector<int> cardOrder = {0, 1, 2, 3};
        std::sort(cardOrder.begin(), cardOrder.end(), [&](int a, int b) {
            return cardinals[a].incomingAngle < cardinals[b].incomingAngle;
        });

        std::vector<int> bestAssignment(nRoads, -1);
        double           bestTotalError = 1e18;

        for (int offset = 0; offset < 4; ++offset) {
            double totalError = 0.0;
            for (int i = 0; i < nRoads; ++i) {
                int cardIdx = cardOrder[(offset + i) % 4];
                totalError += angleDiff(roadAngles[roadOrder[i]],
                                        cardinals[cardIdx].incomingAngle);
            }
            if (totalError < bestTotalError) {
                bestTotalError = totalError;
                for (int i = 0; i < nRoads; ++i)
                    bestAssignment[roadOrder[i]] = cardOrder[(offset + i) % 4];
            }
        }

        return bestAssignment;
    }

    inline CardinalDir oppositeCardinal(CardinalDir d) {
        switch (d) {
            case CardinalDir::North: return CardinalDir::South;
            case CardinalDir::South: return CardinalDir::North;
            case CardinalDir::East:  return CardinalDir::West;
            case CardinalDir::West:  return CardinalDir::East;
        }
        throw std::runtime_error("unreachable");
    }


    // -----------------------------------------------------------------------------
    //  Classify lanes on a single road as seen from its destination intersection.
    //
    //  Runs the same Steps A/B/D logic on the destination intersection JSON to
    //  determine which TurnType each lane of `roadId` takes there.
    //  If the destination is virtual, returns a single Straight entry for all lanes.
    // -----------------------------------------------------------------------------
    inline std::map<TurnType, LaneList> classifyRoadAtDestination(
        json const&        destInterJson,
        std::string const& roadId,
        double             destX,
        double             destY,
        std::unordered_map<std::string, std::array<double,4>> const& roadEndpoints,
        // road_id -> {startX, startY, endX, endY}
        std::unordered_map<std::string, std::string> const& roadEndInter,
        // road_id -> endIntersectionId
        int numLanes
    ) {
        // Virtual destination: all lanes straight.
        if (destInterJson["virtual"].get<bool>()) {
            std::map<TurnType, LaneList> res;
            LaneList& sl = res[TurnType::Straight];
            for (int i = 0; i < numLanes; ++i)
                sl.push_back(roadId + "_" + std::to_string(i));
            std::sort(sl.begin(), sl.end());
            return res;
        }

        std::string destId = destInterJson["id"].get<std::string>();

        // Step A mirror: gather unique inbound roads at dest and approach angles.
        std::vector<std::string> inboundRoads;
        std::vector<double>      inboundAngles;

        for (auto const& rl : destInterJson["roadLinks"]) {
            std::string rid = rl["startRoad"].get<std::string>();
            bool seen = false;
            for (auto const& r : inboundRoads)
                if (r == rid) { seen = true; break; }
            if (seen) continue;

            auto const& ep = roadEndpoints.at(rid);
            // Determine which endpoint is "far" (not the destination intersection).
            double farX, farY;
            if (roadEndInter.at(rid) == destId)
            { farX = ep[0]; farY = ep[1]; }   // far end = road start
            else
            { farX = ep[2]; farY = ep[3]; }   // far end = road end

            inboundRoads.push_back(rid);
            inboundAngles.push_back(std::atan2(destY - farY, destX - farX));
        }

        // Step B mirror: assign inbound roads to cardinals (result unused here,
        // but we need roadWonDir to be consistent with parseTurnType usage below).
        // Actually for outbound classification we only need the roadLinks whose
        // startRoad == roadId, so we can skip the full cardinal assignment and
        // directly read TurnType from the JSON.  The cardinal assignment at the
        // destination is only needed if we wanted to fill MovementMap there, which
        // we do not.  We keep it simple: just scan roadLinks for our road.
        (void)inboundRoads;
        (void)inboundAngles;

        // Step D mirror: collect lane ids per TurnType for roadId only.
        std::map<TurnType, std::unordered_set<std::string>> buckets;

        for (auto const& rl : destInterJson["roadLinks"]) {
            if (rl["startRoad"].get<std::string>() != roadId) continue;

            TurnType turn = detail::parseTurnType(rl["type"].get<std::string>());
            for (auto const& ll : rl["laneLinks"]) {
                int laneIdx = ll["startLaneIndex"].get<int>();
                buckets[turn].insert(roadId + "_" + std::to_string(laneIdx));
            }
        }

        std::map<TurnType, LaneList> res;
        for (auto& [turn, idSet] : buckets) {
            LaneList lanes(idSet.begin(), idSet.end());
            std::sort(lanes.begin(), lanes.end());
            res[turn] = std::move(lanes);
        }

        // Dead-end / no roadLinks for this road: fall back to Straight.
        if (res.empty()) {
            LaneList& sl = res[TurnType::Straight];
            for (int i = 0; i < numLanes; ++i)
                sl.push_back(roadId + "_" + std::to_string(i));
            std::sort(sl.begin(), sl.end());
        }

        return res;
    }
}

// ─────────────────────────────────────────────────────────────────────────────
//  Main loader
// ─────────────────────────────────────────────────────────────────────────────

inline std::map<std::string, IntersectionData>
loadRoadnet(std::string const& roadnetPath) {

    json roadnet = detail::loadJson(roadnetPath);

    // ── 1. Index all roads ────────────────────────────────────────────────────
    struct RoadInfo {
        std::string startIntersectionId, endIntersectionId;
        double startX, startY, endX, endY;
        int    numLanes;
    };
    std::unordered_map<std::string, RoadInfo> roadIndex;
    // Flat helpers passed to classifyRoadAtDestination.
    std::unordered_map<std::string, std::array<double,4>> roadEndpoints;
    // road_id -> endIntersectionId  (used to decide which endpoint is "far")
    std::unordered_map<std::string, std::string> roadEndInter;

    for (auto const& r : roadnet["roads"]) {
        std::string rid = r["id"].get<std::string>();
        RoadInfo ri;
        ri.startIntersectionId = r["startIntersection"].get<std::string>();
        ri.endIntersectionId   = r["endIntersection"].get<std::string>();
        auto const& pts = r["points"];
        ri.startX = pts.front()["x"].get<double>();
        ri.startY = pts.front()["y"].get<double>();
        ri.endX   = pts.back()["x"].get<double>();
        ri.endY   = pts.back()["y"].get<double>();

        if (r.contains("lanes"))
            ri.numLanes = static_cast<int>(r["lanes"].size());
        else if (r.contains("numLanes"))
            ri.numLanes = r["numLanes"].get<int>();
        else
            ri.numLanes = 0;

        roadIndex[rid]    = ri;
        roadEndpoints[rid] = { ri.startX, ri.startY, ri.endX, ri.endY };
        roadEndInter[rid]  = ri.endIntersectionId;
    }


    std::unordered_map<std::string, json> interJsonIndex;
    for (auto const& inter : roadnet["intersections"])
        interJsonIndex[inter["id"].get<std::string>()] = inter;
    // ── 2. Process each intersection ──────────────────────────────────────────
    std::map<std::string, IntersectionData> result;

    for (auto const& inter : roadnet["intersections"]) {

        IntersectionData data;
        data.id        = inter["id"].get<std::string>();
        data.x         = inter["point"]["x"].get<double>();
        data.y         = inter["point"]["y"].get<double>();
        data.isVirtual = inter["virtual"].get<bool>();

        if (data.isVirtual) {
            continue;
        }

        // ── Step A: compute approach angle for every unique inbound road ──────

        std::vector<std::string> inboundRoads;
        std::vector<double>      inboundAngles;

        for (auto const& rl : inter["roadLinks"]) {
            std::string rid = rl["startRoad"].get<std::string>();

            bool seen = false;
            for (auto const& r : inboundRoads)
                if (r == rid) { seen = true; break; }
            if (seen) continue;

            auto const& ri = roadIndex.at(rid);

            double farX, farY;
            if (ri.endIntersectionId == data.id)
                { farX = ri.startX;  farY = ri.startY; }
            else
                { farX = ri.endX;    farY = ri.endY;   }

            double dx = data.x - farX;
            double dy = data.y - farY;

            inboundRoads.push_back(rid);
            inboundAngles.push_back(std::atan2(dy, dx));
        }

        // ── Step B: assign inbound roads to cardinals ─────────────────────────

        auto const& table = detail::cardinalTable();
        std::vector<int> assignment =
            detail::assignRoadsToCardinals(inboundAngles, table);

        std::unordered_map<int, std::string>      inboundRoadForCardinal;
        std::unordered_map<std::string, CardinalDir> roadWonDir;

        for (int i = 0; i < static_cast<int>(inboundRoads.size()); ++i) {
            if (assignment[i] < 0) continue;
            CardinalDir dir = table[assignment[i]].dir;
            inboundRoadForCardinal[static_cast<int>(dir)] = inboundRoads[i];
            roadWonDir[inboundRoads[i]] = dir;
        }

        // ── Step C: build the roads map ───────────────────────────────────────

        std::vector<std::string> attachedRoads;
        for (auto const& rid : inter["roads"])
            attachedRoads.push_back(rid.get<std::string>());

        std::unordered_set<std::string> inboundSet(inboundRoads.begin(),
                                                    inboundRoads.end());

        std::vector<std::string> outboundRoads;
        std::vector<double>      outboundAngles;

        for (auto const& rid : attachedRoads) {
            if (inboundSet.count(rid)) continue;

            auto const& ri = roadIndex.at(rid);

            double farX, farY;
            if (ri.startIntersectionId == data.id)
                { farX = ri.endX;    farY = ri.endY;   }
            else
                { farX = ri.startX;  farY = ri.startY; }

            double dx = farX - data.x;
            double dy = farY - data.y;

            outboundRoads.push_back(rid);
            outboundAngles.push_back(std::atan2(dy, dx));
        }

        std::vector<int> outAssignment =
            detail::assignRoadsToCardinals(outboundAngles, table);

        std::unordered_map<int, std::string> outboundRoadForCardinal;
        for (int i = 0; i < static_cast<int>(outboundRoads.size()); ++i) {
            if (outAssignment[i] < 0) continue;
            CardinalDir rawDir  = table[outAssignment[i]].dir;
            CardinalDir realDir = detail::oppositeCardinal(rawDir);
            outboundRoadForCardinal[static_cast<int>(realDir)] = outboundRoads[i];
        }

        for (int dirInt = 0; dirInt < 4; ++dirInt) {
            CardinalDir dir = static_cast<CardinalDir>(dirInt);
            auto inIt  = inboundRoadForCardinal.find(dirInt);
            auto outIt = outboundRoadForCardinal.find(dirInt);
            if (inIt  != inboundRoadForCardinal.end())
                data.roads[{dir, RoadDir::Inbound}]  = inIt->second;
            if (outIt != outboundRoadForCardinal.end())
                data.roads[{dir, RoadDir::Outbound}] = outIt->second;
        }

        // ── Step D: populate movement buckets with lane ids ───────────────────

        std::unordered_map<MovementKey, std::unordered_set<std::string>,
                           MovementKeyHash> bucketSets;

        for (auto const& rl : inter["roadLinks"]) {
            std::string startRoadId = rl["startRoad"].get<std::string>();

            auto it = roadWonDir.find(startRoadId);
            if (it == roadWonDir.end()) continue;

            CardinalDir approachDir = it->second;
            TurnType    turn        = detail::parseTurnType(
                                          rl["type"].get<std::string>());
            MovementKey key{ approachDir, turn };

            for (auto const& ll : rl["laneLinks"]) {
                int laneIdx = ll["startLaneIndex"].get<int>();
                bucketSets[key].insert(startRoadId + "_" +
                                       std::to_string(laneIdx));
            }
        }

        for (auto& [key, idSet] : bucketSets) {
            LaneList lanes(idSet.begin(), idSet.end());
            std::sort(lanes.begin(), lanes.end());
            data.movements[key] = std::move(lanes);
        }

        // ── Step E: build phaseLanes ──────────────────────────────────────────
        //
        // trafficLight.roadLinkIndices lists which roadLink slots participate
        // in the traffic light at all.
        //
        // trafficLight.lightphases[i].availableRoadLinks lists indices INTO
        // roadLinkIndices (not directly into roadLinks) that are green in
        // phase i.
        //
        // So to get the actual roadLink for a phase entry k:
        //   roadLinks[ roadLinkIndices[k] ]
        //
        // From each active roadLink we collect the unique startLaneIndex values
        // and format them as lane ids exactly like we do for movements.

        auto const& tl              = inter["trafficLight"];
        auto const& roadLinkIndices = tl["roadLinkIndices"];
        auto const& lightPhases     = tl["lightphases"];
        auto const& roadLinks       = inter["roadLinks"];

        for (int phaseIdx = 0;
             phaseIdx < static_cast<int>(lightPhases.size()); ++phaseIdx)
        {
            std::unordered_set<std::string> laneSet;

            // For phasesData: accumulate lanes per (inboundRoad|outboundRoad).
            std::unordered_map<std::string,
                std::unordered_set<std::string>> movLanes;

            for (auto const& rlIdxJson :
                     lightPhases[phaseIdx]["availableRoadLinks"])
            {
                int rlIndirect  = rlIdxJson.get<int>();
                int rlDirect    = roadLinkIndices[rlIndirect].get<int>();

                auto const& rl      = roadLinks[rlDirect];
                std::string startRoadId = rl["startRoad"].get<std::string>();
                std::string endRoadId   = rl["endRoad"].get<std::string>();

                for (auto const& ll : rl["laneLinks"]) {
                    int laneIdx = ll["startLaneIndex"].get<int>();
                    std::string laneId = startRoadId + "_" +
                                         std::to_string(laneIdx);
                    laneSet.insert(laneId);
                    movLanes[startRoadId + "|" + endRoadId].insert(laneId);
                }
            }

            // phaseLanes entry
            LaneList pll(laneSet.begin(), laneSet.end());
            std::sort(pll.begin(), pll.end());
            data.phaseLanes[phaseIdx] = std::move(pll);

            // phasesData entry
            PhaseData pd;
            for (auto& [key, idSet] : movLanes) {
                auto sep = key.find('|');
                TurnMovement tm;
                tm.inboundRoadId  = key.substr(0, sep);
                tm.outboundRoadId = key.substr(sep + 1);
                tm.inboundLaneIds = LaneList(idSet.begin(), idSet.end());
                std::sort(tm.inboundLaneIds.begin(), tm.inboundLaneIds.end());
                pd.movements.push_back(std::move(tm));
            }
            std::sort(pd.movements.begin(), pd.movements.end(),
                [](TurnMovement const& a, TurnMovement const& b) {
                    if (a.inboundRoadId != b.inboundRoadId)
                        return a.inboundRoadId < b.inboundRoadId;
                    return a.outboundRoadId < b.outboundRoadId;
                });
            data.phasesData.push_back(std::move(pd));
        }

        // ── Step F: build roadTurnLanes ───────────────────────────────────────
        //
        // Inbound roads: read turn type directly from this intersection's
        // movement buckets (same logic as original).
        //
        // Outbound roads: open the destination intersection's JSON and run
        // the same roadLink scan to classify each lane of the outbound road.
        // If the destination is virtual, all lanes are Straight.

        // Inbound roads.
        for (auto const& [mvKey, lanes] : data.movements) {
            auto roadIt = data.roads.find({mvKey.from, RoadDir::Inbound});
            if (roadIt == data.roads.end()) continue;

            std::string const& roadId = roadIt->second;
            auto& laneList = data.roadTurnLanes[roadId][mvKey.turn];
            laneList.insert(laneList.end(), lanes.begin(), lanes.end());
        }
        for (auto& [roadId, turnMap] : data.roadTurnLanes)
            for (auto& [turn, laneList] : turnMap)
                std::sort(laneList.begin(), laneList.end());

        // Outbound roads.
        for (auto const& outRoadId : outboundRoads) {
            auto const& ri = roadIndex.at(outRoadId);

            // Destination intersection for this outbound road.
            std::string destInterId =
                (ri.startIntersectionId == data.id)
                    ? ri.endIntersectionId
                    : ri.startIntersectionId;

            auto const& destJson = interJsonIndex.at(destInterId);
            double destX = destJson["point"]["x"].get<double>();
            double destY = destJson["point"]["y"].get<double>();

            auto turnLanes = detail::classifyRoadAtDestination(
                destJson,
                outRoadId,
                destX, destY,
                roadEndpoints,
                roadEndInter,
                ri.numLanes
            );

            for (auto& [turn, lanes] : turnLanes) {
                auto& dst = data.roadTurnLanes[outRoadId][turn];
                dst.insert(dst.end(), lanes.begin(), lanes.end());
                std::sort(dst.begin(), dst.end());
            }
        }

        result[data.id] = std::move(data);
    }

    return result;
}

// ─────────────────────────────────────────────────────────────────────────────
//  Config-based entry point
// ─────────────────────────────────────────────────────────────────────────────

inline std::map<std::string, IntersectionData>
loadFromConfig(std::string const& configPath) {
    json cfg = detail::loadJson(configPath);

    std::string dir         = cfg.value("dir", "");
    std::string roadnetFile = cfg["roadnetFile"].get<std::string>();

    if (!dir.empty() && dir.back() != '/' && dir.back() != '\\')
        dir += '/';

    return loadRoadnet(dir + roadnetFile);
}