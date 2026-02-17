#include "AStar.h"
#include <vector>  // std::vector
#include <utility> // std::pair, std::make_pair
#include <numeric>  // std::iota 連続した整数を簡単に入れる関数　手動ループ省略
#include <cmath>
#include <cstdio>
#include <algorithm>
#include <iostream>

AStar::AStar() {}

extern coords gPosi;
extern field cubePosi;
extern field3 cubePosi3;
extern field tar_cubePosi;
extern field3 tar_cubePosi3;

extern Obstacle objePosi[12];
extern Obstacle objePosi2[12];

///A*疑似アルゴリズムver.1---------------------------------------
//ヒューリスティック 
double AStar::heuristic(int x, int y, int gx, int gy) {
    // return sqrt((x - gx) * (x - gx) + (y - gy) * (y - gy));//（ユークリッド距離）
    return abs(x - gx) + abs(y - gy);//（マンハッタン距離）
}

// ---- コスト設定 ----
const double COST_MOVE      = 1.0;//距離
const double COST_TURN      = 0.0;//旋回数
const double COST_STEP_UP   = 0.0;//秒数など
const double COST_STEP_DOWN = 0.0;

//隣接移動
std::vector<std::pair<int,int>> AStar::motionModel() {
    return { {1,0}, {-1,0}, {0,1}, {0,-1} };//前，後，右，左
}

// ゴール判定関数
// bool AStar::isGoal(int x, int y) {
//     for(auto g : goals) {
//         if (x == g.first && y == g.second) return true;
//     }
//     return false;
// }
bool AStar::isGoal(int x, int y,
                   const std::vector<std::pair<int,int>>& goals)
{
    for (auto& g : goals) {
        if (x == g.first && y == g.second) return true;
    }
    return false;
}

//同じ座標判定（ゴール判定）
bool AStar::isSamePosi(int x1, int y1, int x2, int y2) {
    return (x1 == x2 && y1 == y2);
}

//障害物判定
bool AStar::isObstacle(int x, int y, const std::vector<std::pair<int,int>>& obstacles){
    for (auto& o : obstacles) {//auto:型の推論，auto&：要素への参照
        if (isSamePosi(x, y, o.first, o.second)) return true;
    }
    return false;
}

int AStar::getCollectedCount(){
    return collect_count;
}

// A*本体
std::vector<std::pair<int,int>> AStar::AStar1(
    int sx, int sy, int gx, int gy,
    const std::vector<std::pair<int,int>>& obstacles,
    int xmax, int ymax)
{
    std::vector<Node> open, close;
    std::vector<std::pair<int,int>> path;
    
    Node start = {sx, sy, 0, 0, 0, heuristic(sx,sy,gx,gy), sx, sy, 0, UP_dir, 0};
    start.f = start.g + start.h;
    open.push_back(start);

    while (!open.empty()) {
        // f値でソート
        std::sort(open.begin(), open.end(),
                  [](const Node& a, const Node& b){ return a.f < b.f; });

        Node current = open.front();
        open.erase(open.begin());
        close.push_back(current);

        // ゴール判定
        if (isSamePosi(current.x, current.y, gx, gy)) {
        // if (isGoal(current.x, current.y)) {
            // printf("Goal found!!\n");
            // 経路復元
            path.push_back({current.x, current.y});
            bool pathFound = true;
            while (!(isSamePosi(current.x, current.y, sx, sy))) {//ここでループしちゃう
                bool parentFound = false;
                //  printf("%d,%d,%d,%d\n",current.x, current.y, sx, sy); 
                for (auto& n : close) {
                    if (isSamePosi(n.x, n.y, current.px, current.py)) {
                        current = n;
                        path.push_back({current.x, current.y});
                        parentFound = true;
                        break;
                    }
                }
                 if (!parentFound) {
                    // printf("Parent found!!\n");     
                    pathFound = false;
                    break; 
                }
            }
            std::reverse(path.begin(), path.end());
            return path;
        }

        // 隣接ノード探索
        for (auto& d : motionModel()) {
            int nx = current.x + d.first;
            int ny = current.y + d.second;
            int collectedCount = getCollectedCount();

            if (nx < -1 || nx > xmax || ny < -1 || ny > ymax) continue;//フィールド条件
            if (isObstacle(nx, ny, obstacles)) continue;//障害物があった時
            if(nx == -1) continue;
            // if (collectedCount < 3 && nx == 4) continue;
            // if (nx == 4 && collectedCount < limit_collect_num) continue;
           
            // if (isGateCell(nx, ny) && !gateOpen) {
            //     continue; // まだ通れない
            // }
            // if (nx == 4 && collectedCount < targetCount) {
            //     continue;
            // }


            //旋回ペナルティ
            int turnPenalty = 0;
            if (current.dir != -1 ) {
                turnPenalty = 1;
            }
            //沼ペナルティ
            int swampPenalty = 0;
            // if (nx >= 0 && nx <= 3 && ny >= 0 && ny <= 2) {
            //     swampPenalty = 2;  // 数字調整する
            // }
            if(nx == -1){
                swampPenalty = 2;
            }
            //forestを出ないため(回収数で無理やり変更しているが，こっちでコストを高くしてもいい)
            int goallinePenalty = 0;
            // if(nx == 4 ){
            if (nx == 4 && collectedCount < limit_collect_num){
              goallinePenalty = 10;//複数ゴールにしたからこれいらないかも
            }

            //段差計算
            int hCur = heightMap[current.y][current.x];
            int hNext = heightMap[ny][nx];
            int dh = hNext - hCur;

            //段差ペナルティ
            int slopeCost = 0;
            if(dh > 0){
                slopeCost = COST_UP * dh;
            } else if(dh < 0){
                slopeCost = COST_DOWN * (-dh);
            }

            // 合計コスト
            double g = current.g + 1 + turnPenalty + swampPenalty + slopeCost + goallinePenalty;
            double h = heuristic(nx, ny, gx, gy);
            // double h = heuristic_dir(nx, ny, gx, gy, -turn_cost);
            double f = g + h;
           
            // open にあるかチェック
            bool skip = false;
            for (auto& n : open) {
                if (isSamePosi(n.x, n.y, nx, ny) && f >= n.f) {
                    skip = true; break;
                }
            }
            if (skip) continue;

            // close にあるかチェック
            for (auto& n : close) {
                if (isSamePosi(n.x, n.y, nx, ny) && f >= n.f) {
                    skip = true; break;
                }
            }
            if (skip) continue;

            // Node newNode = {nx, ny, f, g, h, current.x, current.y};
            
            Node newNode = {nx, ny, 0, f, g, h, current.x, current.y, 0, 0, 0};
            open.push_back(newNode);
        }
    }
    // printf("No path to goal!!\n");
    return path;
}

std::vector<std::pair<int,int>> AStar::autoSelectAStar(
    int sx, int sy,
    // int gx, int gy,
    const std::vector<std::pair<int,int>>& goals,
    const std::vector<std::pair<int,int>>& candidates,
    const std::vector<std::pair<int,int>>& obstacles,
    int xmax, int ymax,
    int minPass,
    int maxDistance) // ← 総距離の上限
{
    int n = candidates.size();
    double bestCost = 1e9;
    std::vector<int> bestOrder;

    //候補点の部分集合を作る．n個の候補点から，どの点を回るかを決める（TSPでいうどの都市を訪問するかという部分）
    for (int mask = 0; mask < (1 << n); mask++) {
        std::vector<int> subset;
        for (int i = 0; i < n; i++)
            if (mask & (1 << i)) subset.push_back(i);
            
        //printf("subset size: %zu, indices:", subset.size());
        if (subset.size() < minPass) continue;// 最低通過点未満はスキップ スルーしないように
        if (through_flag) continue;

        std::sort(subset.begin(), subset.end());
        do {
            double total = 0;//合計距離
            int cx = sx, cy = sy;
            bool failed = false;

            for (int idx : subset) {
                auto path = AStar1(cx, cy, candidates[idx].first, candidates[idx].second,
                                  obstacles, xmax, ymax);
                if (path.empty()) { failed = true; break; }
                total += path.size();
                cx = candidates[idx].first;
                cy = candidates[idx].second;
            }
            //printf("size3: %lf, size4: %lf\n", total_3, total_4);

    //-------------------------------------------------------
            double bestGoalCost = 1e9;
            std::vector<std::pair<int,int>> bestGoalPath;

            for (auto &g : goals) {
                auto p = AStar1(cx, cy, g.first, g.second, obstacles, xmax, ymax);
                if (!p.empty() && p.size() < bestGoalCost) {
                    bestGoalCost = p.size();
                    bestGoalPath = p;
                }
            }

            if (bestGoalPath.empty()) continue;
            total += bestGoalCost;

            // auto pathToGoal = AStar1(cx, cy, gx, gy, obstacles, xmax, ymax);
            // if (pathToGoal.empty()) continue;
            // total += pathToGoal.size();

            //printf("total:%lf\n",total);

            if (!failed) {
                // 条件チェック
                // if (subset.size() > minPass) continue; 
                if (total < bestCost) {
                    bestCost = total;
                    bestOrder = subset;
                }
            }
        } while (std::next_permutation(subset.begin(), subset.end()));
    }

    // 最適順序で経路を結合(TSPみたいなやつ) 
    std::vector<std::pair<int,int>> fullPath;
    int cx = sx, cy = sy;
    for (int idx : bestOrder) {
        auto path = AStar1(cx, cy, candidates[idx].first, candidates[idx].second,
                          obstacles, xmax, ymax);
        if (!fullPath.empty()) fullPath.insert(fullPath.end(), path.begin()+1, path.end());
        else fullPath.insert(fullPath.end(), path.begin(), path.end());
        cx = candidates[idx].first;
        cy = candidates[idx].second;
    }

    // auto pathToGoal = AStar1(cx, cy, gx, gy, obstacles, xmax, ymax);
    // if (!fullPath.empty()) fullPath.insert(fullPath.end(), pathToGoal.begin()+1, pathToGoal.end());
    // else fullPath.insert(fullPath.end(), pathToGoal.begin(), pathToGoal.end());
    double bestGoalCost = 1e9;
    std::vector<std::pair<int,int>> bestGoalPath;

    for (auto &g : goals) {
        auto p = AStar1(cx, cy, g.first, g.second, obstacles, xmax, ymax);
        if (!p.empty() && p.size() < bestGoalCost) {
            bestGoalCost = p.size();
            bestGoalPath = p;
        }
    }

    if (!bestGoalPath.empty()) {
        fullPath.insert(fullPath.end(),
            bestGoalPath.begin() + (fullPath.empty() ? 0 : 1),
            bestGoalPath.end());
    }


    return fullPath;
}

void AStar::addCandidate(int x, int y) {
    candidates.emplace_back(x, y); // pair(x, y) を vector に追加
}
void AStar::clearCandidates() {
    candidates.clear();
}
void AStar::clearObstacles() {
    obstacles.clear();
}
void AStar::clearUncollect() {
    uncollectedR1.clear();
}

//重複防止用
void AStar::addObstacleIfNew(int x, int y){
    for(const auto &p : obstacles){
        if(p.first == x && p.second == y) return; // 重複は追加しない
    }
    // printf("addobj");
    obstacles.push_back({x,y});
}

void AStar::addUncollectIfNew(int x, int y){
    for(const auto &p : uncollectedR1){
        if(p.first == x && p.second == y) return; // 重複は追加しない
    }
    // printf("addobj");
    uncollectedR1.push_back({x,y});
}

void AStar::addUncollectindicesIfNew(int i){
    for(const auto &idx : uncollectedR1_indices){
        if(idx == i) return; // 重複は追加しない
    }
    // printf("addobj");
    uncollectedR1_indices.push_back(i);
}

// 重複のないように candidate を追加する関数
void AStar::addCandidateIfNew(int x, int y){
    for(auto &p : candidates){
        if(p.first == x && p.second == y) return;
    }
    candidates.push_back({x, y});
}

void AStar::updateCandidates(bool obj[], int size){
    for(int i = 0; i < size; i++){
        if(obj[i]){
            addCandidateIfNew(objePosi2[i].x, objePosi2[i].y);
        }
    }
}

//隣接するマスにあるR2KFS(取得物)の個数カウント
int AStar::countAdjacentR2KFS(int cx, int cy,
                       const std::vector<std::pair<int,int>>& candidates,
                       size_t n)
{
    // 上下左右4方向に1マス
    const int dx[4] = { 1, -1, 0, 0 };
    const int dy[4] = { 0, 0, 1, -1 };

    int count = 0;

    for(int i = 0; i < 4; i++){
        int nx = cx + dx[i];
        int ny = cy + dy[i];

        // 全オブジェクトに対し一致しているかチェックを行う．
        for (int j = 0; j < n; j++) {
            if (candidates[j].first == nx &&
                candidates[j].second == ny) {
                count++;
            }
        }
    }
    return count;
}

void AStar::make_plusdist_stepwise() {
    empty_flag = false;
    int minPass = candidates.size();
    // int minPass = 4;
    // limit_collect_num = 4;
    int maxDistance = 100;
    
    directions.clear(); // 前回の指示をクリア
    dist_plus_x = 0.0;
    dist_plus_y = 0.0;
    tar_posi_x = gPosi.x;
    tar_posi_y = gPosi.y;
    // total = 0;

    

    // gateOpen = (collect_count >= limit_collect_num);


    for (int i = 0; i < 12; i++) {
        // もし現在位置がその物体の座標と一致していたら
        if (cubePosi.x == objePosi3[i].x && cubePosi.y == objePosi3[i].y) {
            // その obj_true[i] が false になるまで待つ
            if (R2KFS_posi[i]) {
                wait_id = i;
                // printf("waitid:%d",wait_id);
                dist_plus_x = 0.0;
                dist_plus_y = 0.0;
                tar_posi_x = gPosi.x;
                tar_posi_y = gPosi.y;
                waitobj = true;
                // return; // このステップの更新をスキップ
            }else if(!R2KFS_posi[i]){
                waitobj = false;
            }
            // if(R2KFS_posi[i] == false && pre_R2KFS_posi[i] == true){
            //     static int count = 0;
            //     count++;
            //     collect_count = count / 2;
            // }
        }
    }
    // printf("waitid:%d",wait_id);
    // printf("wait:%d",waitobj);
   
    bool reachedGoal = false;
    for (auto &g : goals) {
        if (cubePosi.x == g.first && cubePosi.y == g.second) {
            reachedGoal = true;
            break;
        }
    }

    // 現在位置からゴールまでループ
    // if(!(cubePosi.x == tar_cubePosi.x && cubePosi.y == tar_cubePosi.y)) {
    if(!reachedGoal){
        clearCandidates();
        clearObstacles();
        clearUncollect();
        uncollectedR1.clear();
        uncollectedR1_indices.clear();
        
        // if (collect_count < limit_collect_num ) {
        //     // for (int y = 0; y < 2; ++y)
        //         // addObstacleIfNew(4, 0);
        //         // addObstacleIfNew(4, 1);
        //         addObstacleIfNew(4, 2);
        // }else if(collect_count == limit_collect_num){
        //     int diffPosi1 = heuristic(cubePosi.x, cubePosi.y, 4, 0);
        //     int diffPosi2 = heuristic(cubePosi.x, cubePosi.y, 4, 1);
        //     int diffPosi3 = heuristic(cubePosi.x, cubePosi.y, 4, 2);
        //     if(diffPosi1 < diffPosi2 && diffPosi1 < diffPosi2){
        //         tar_cubePosi = {4, 0};
        //     }else if(diffPosi2 < diffPosi1 && diffPosi2 < diffPosi3){
        //         tar_cubePosi = {4, 1};
        //     }else if(diffPosi3 < diffPosi1 && diffPosi3 < diffPosi2){
        //         tar_cubePosi = {4, 2};
        //     }else {
        //         tar_cubePosi = {4, 1};
        //     }
        //     collect_count++;
        // }

        for (int i = 0; i < 12; ++i) {
            addObstacleIfNew(obstacle12[i].x, obstacle12[i].y);
            // if (obj_true[i]) {
            //     // addCandidateIfNew(objePosi3[i].x, objePosi3[i].y);
            //     addObstacleIfNew(objePosi3[i].x, objePosi3[i].y);
            // }
            if (R1KFS_posi[i]) {
                addObstacleIfNew(objePosi3[i].x, objePosi3[i].y);
                // addUncollectIfNew(objePosi3[i].x, objePosi3[i].y);//R1除外用
            }
            // 表示する
            // std::cout << "R1未回収物座標順序:" << std::endl;
            for(auto &p : uncollectedR1){
                // std::cout << "(" << p.first << "," << p.second << ")" << std::endl;
                uncole1 = p.first;
                uncole2 = p.second;
            }
            
            if(Fake_posi[i]){
                Fake_posix = objePosi3[i].x;
                Fake_posiy = objePosi3[i].y;
                addObstacleIfNew(objePosi3[i].x, objePosi3[i].y);
            }
            
            if (R2KFS_posi[i]) {
                addCandidateIfNew(objePosi3[i].x, objePosi3[i].y);
            }
            // if(obj_true1[i] == 1 || obj_true1[i] == 3){
            //     addObstacleIfNew(objePosi3[i].x, objePosi3[i].y);
            // }
            // if(obj_true1[i] == 2){
            //     addCandidateIfNew(objePosi3[i].x, objePosi3[i].y);
            // }
        }

        if(collect_count == 4){//4個回収したらクリアする
            clearCandidates();
        }

        R2_Adjacent_count = countAdjacentR2KFS(cubePosi.x, cubePosi.y, candidates, candidates.size());
        if(R2_Adjacent_count >= 2){
            samePosi_flag_Adjacent = true;
        }else {
            samePosi_flag_Adjacent = false;
        }
    
        // 今の位置から A* 経路探索
        // auto path = autoSelectAStar(cubePosi.x, cubePosi.y,
        //                              tar_cubePosi.x, tar_cubePosi.y,
        //                              candidates, obstacles,
        //                              forestXYMAX.x, forestXYMAX.y,
        //                              minPass, maxDistance);
         // 外周障害物を無視して再計算---------------------//障害物の除去を行うと避けて進む動作ができなくなる．→目標回数以下の時には除去する．それ以外は
        std::vector<std::pair<int,int>> reduced_obstacles;
        for (auto &obs : obstacles) {
            // 外周だけを除外
            bool isOuter = (obs.first == 0 || obs.first == 3 ||//除外する行
                            obs.second == 0 || obs.second == 2);
            bool isFake = (obs.first == Fake_posix && obs.second == Fake_posiy);//除外しない場所
            if (!isOuter || isFake) reduced_obstacles.push_back(obs);
        }
        //----------------------------------
        // if(collect_count < limit_collect_num){//取得上限になったら障害物をよけながら進む．R2KFSは近くからとっていくのでそれまでにR1がとってくれる想定の時
        //     obstacles = reduced_obstacles;  //R1が止まったりしているときはこっちを捨てていくしかない⇒障害物にKFSが埋められているときはそれを取れないため
        // }
            auto path = autoSelectAStar(cubePosi.x, cubePosi.y,
                                    goals,
                                    candidates, obstacles,
                                    forestXYMAX.x, forestXYMAX.y,
                                    minPass, maxDistance);

        int diff_dist = 10;
        total = path.size();
        if(total > diff_dist){
            limit_collect_num = 3;
        }else{
            limit_collect_num = 4;   
        }
        // uncollectedR1.clear();
        // uncollectedR1_indices.clear();
        for (auto &pos : path) {
            for (int i = 0; i < 12; i++) {
                if (R1KFS_posi[i] &&
                    pos.first == objePosi3[i].x &&
                    pos.second == objePosi3[i].y) {
                    addUncollectIfNew(objePosi3[i].x, objePosi3[i].y);  // R1の座標を順番に追加
                    // uncollectedR1_indices.push_back(i);  // R1番号を順番に追加
                    addUncollectindicesIfNew(i);
                    // R1KFS_posi[i] = false;   // 重複防止
                }
            }
        }
        
        if (path.empty()) {
            // std::cout << "No path to goal blocked by obstacles!" << std::endl;
            // dist_plus_x = 0.0;
            // dist_plus_y = 0.0;
            empty_flag = true;

             // 外周障害物を無視して再計算
            std::vector<std::pair<int,int>> reduced_obstacles;
            for (auto &obs : obstacles) {
                // 外周だけを除外
                bool isOuter = (obs.first == 0 || obs.first == 3 ||//除外する行
                                obs.second == 0 || obs.second == 2);
                bool isFake = (obs.first == Fake_posix && obs.second == Fake_posiy);//除外しない場所
                if (!isOuter || isFake) reduced_obstacles.push_back(obs);
            }

            auto path2 = autoSelectAStar(cubePosi.x, cubePosi.y,
                                    goals,
                                    candidates, reduced_obstacles,
                                    forestXYMAX.x, forestXYMAX.y,
                                    minPass, maxDistance);
            
            // uncollectedR1.clear();
            // uncollectedR1_indices.clear();
            for (auto &pos : path) {
                for (int i = 0; i < 12; i++) {
                    if (R1KFS_posi[i] &&
                        pos.first == objePosi3[i].x &&
                        pos.second == objePosi3[i].y) {
                        addUncollectIfNew(objePosi3[i].x, objePosi3[i].y);  // R1の座標を順番に追加
                        // uncollectedR1_indices.push_back(i);  // R1番号を順番に追加
                        addUncollectindicesIfNew(i);
                        next_indices = i;
                        // R1KFS_posi[i] = false;   // 重複防止
                    }
                }
            }
            // for (auto &index : uncollectedR1_indices) {
            //     next_indices = index;
            // }
            // for (auto &p : uncollectedR1) {
            //     std::cout << "(" << p.first << "," << p.second << ")" << std::endl;
            // }

            // for (auto &idx : uncollectedR1_indices) {
            //     std::cout << "R1番号: " << idx << std::endl;
            // }



            total = path2.size();
            if (!path2.empty()) {
                // 経路があった場合、外周にまだR1が回収していない物体があるか確認
                int stopIndex = path2.size(); // デフォルトは最後まで進む
                for (size_t i = 0; i < path2.size(); ++i) {
                    int px = path2[i].first;
                    int py = path2[i].second;
                    for (int j = 0; j < 12; ++j) {
                        bool isOuter = (objePosi3[j].x == 0 || objePosi3[j].x == 3 ||
                                        objePosi3[j].y == 0 || objePosi3[j].y == 2 );
                        bool isFake = (objePosi3[j].x == Fake_posix && objePosi3[j].y == Fake_posiy);
                        if (isOuter && R1KFS_posi[j]) { // R1がまだ回収していない
                            if (px == objePosi3[j].x && py == objePosi3[j].y) {
                                stopIndex = i;
                                break;
                            }
                        }
                    }
                    if (stopIndex != path2.size()) break;
                }
                // empty_total = stopIndex-1;
                // stopIndex のマスまで移動
                if (cubePosi.x == path2[stopIndex-1].first &&
                    cubePosi.y == path2[stopIndex-1].second) {
                    nextX = path2[1].first;
                    nextY = path2[1].second;
                    int dx = nextX - cubePosi.x;
                    int dy = nextY - cubePosi.y;
                    dist_plus_x = 1.2 * dx;
                    dist_plus_y = 1.2 * dy;
                    // dist_plus_x = 0.0;
                    // dist_plus_y = 0.0;
                    tar_posi_x = gPosi.x;
                    tar_posi_y = gPosi.y;
                    empty_flag = true; // 待機状態
                } else {
                    nextX = path2[1].first;
                    nextY = path2[1].second;
                    nextX2 = path2[2].first;
                    nextY2 = path2[2].second;
                    int dx = nextX - cubePosi.x;
                    int dy = nextY - cubePosi.y;
                    directions.push_back({dx, dy});
                    if(cubePosi.x == nextX2 && cubePosi.y == nextY2 || samePosi_flag_Adjacent){//次の次が同じ位置だったらまたは隣接マスに2個以上あったら移動しきらない
                        dist_plus_x = 0.4 * dx;
                        dist_plus_y = 0.4 * dy;
                        samePosi_flag = true;
                    }else{
                        dist_plus_x = 1.2 * dx;
                        dist_plus_y = 1.2 * dy;
                        samePosi_flag = false;
                    }
                    
                    dx_ = dx;
                    dy_ = dy;
                    empty_flag = false;
                }
                // int dx = nextX - cubePosi.x;
                // int dy = nextY - cubePosi.y;
                // directions.push_back({dx, dy});
                // dist_plus_x = 1.2 * dx;
                // dist_plus_y = 1.2 * dy;
                // dx_ = dx;
                // dy_ = dy;
                // empty_flag = false;
            } else {
                // それでも塞がってる場合は待機
                // dist_plus_x = 0.0;
                // dist_plus_y = 0.0;
                tar_posi_x = gPosi.x;
                tar_posi_y = gPosi.y;
                empty_flag = true;
            }

        }else{
            empty_flag = false;
            total = path.size();
            //次の1マスだけ進む
            nextX = path[1].first;
            nextY = path[1].second;
            nextX2 = path[2].first;
            nextY2 = path[2].second;
            // 方向ベクトルを記録
            int dx = nextX - cubePosi.x;
            int dy = nextY - cubePosi.y;
            // printf("nextX:%d,nextY:%d ",nextX,nextY);
            // printf("dx:%d,dy:%d ",dx,dy);
            // printf("Cubex:%d,y:%d",cubePosi.x,cubePosi.y);
            directions.push_back({dx, dy});
             if(cubePosi.x == nextX2 && cubePosi.y == nextY2 || samePosi_flag_Adjacent){
                dist_plus_x = 0.4 * dx;
                dist_plus_y = 0.4 * dy;
                samePosi_flag = true;
            }else{
                dist_plus_x = 1.2 * dx;
                dist_plus_y = 1.2 * dy;
                samePosi_flag = false;
            }
            //現在位置を更新
            // cubePosi.x = nextX;
            // cubePosi.y = nextY;
            dx_ = dx;
            dy_ = dy;
        }

        int index_nextPosi = 0;
        int index_nowPosi = 0;
        for(int i = 0; i < 18; i++){
            if(nextX == objePosi5[i].x && nextY == objePosi5[i].y){
                index_nextPosi = i;
            }
            if(cubePosi.x == objePosi5[i].x && cubePosi.y == objePosi5[i].y){
                index_nowPosi = i;
            }
        }
        tar_posi_x = forestPosi5[index_nextPosi].x;
        tar_posi_y = forestPosi5[index_nextPosi].y;
        now_posi_x = forestPosi5[index_nowPosi].x;
        now_posi_y = forestPosi5[index_nowPosi].y;

    }else {
        dist_plus_x = 0.0;
        dist_plus_y = 0.0;
        tar_posi_x = gPosi.x;
        tar_posi_y = gPosi.y;
        goal_flag = true;
    }
}