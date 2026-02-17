#ifndef ASTAR_h
#define ASTAR_h

#include "define.h"
#include <vector> 

class AStar{
    public:
    AStar();
                
    std::vector<std::pair<int,int>> AStar1(
        int sx, int sy, int gx, int gy,
        const std::vector<std::pair<int,int>>& obstacles,
        int xmax, int ymax);
    std::vector<std::pair<int,int>> autoSelectAStar(
        int sx, int sy,
        // int gx, int gy,
        const std::vector<std::pair<int,int>>& goals,
        const std::vector<std::pair<int,int>>& candidates,
        const std::vector<std::pair<int,int>>& obstacles,
        int xmax, int ymax,
        int minPass,
        int maxDistance);
    //ヒューリスティック関数
    double heuristic(int x, int y, int gx, int gy);//MODE:2
    std::vector<std::pair<int,int>> motionModel();
    bool isObstacle(int x, int y, const std::vector<std::pair<int,int>>& obstacles);//障害物判定
    bool isSamePosi(int x1, int y1, int x2, int y2);//スタート・ゴール判定
    int countAdjacentR2KFS(int cx, int cy,
                       const std::vector<std::pair<int,int>>& candidates,
                       size_t n);

    int R2_Adjacent_count = 0;
    double evalParam = {};
    void set_point();
    void make_plusdist();
    void make_plusdist_stepwise();//AutoControlで使うやつ
    void make_plusdist_stepwise3();//AutoControlで使うやつ 3次元用

    int turn_cost = 0;
        //A*アルゴリズム用
    int dir_num = 0;//上下左右への移動方向
    int total = 0;//作成されるpathの数
    int steps = 0;//ステップ数
    int empty_total = 0;
    int empty_steps = 0;
    int next_dir[3];
    int nObstacle = 4;//障害物の数
    int obstacle[2];
    bool find_flag = false;//ゴールの発見フラグ
    bool obstacle_flag = false;//経路上に障害物あるなしのフラグ
    bool move_check_flag = false;//次のマスに進んでいいかのフラグ
    bool goal_flag = false;
    
    int count_R2 = 0;
    int obs_i = 0;
    int collect_i = 0;

    //障害物MATLABだとgetobstacle
    Obstacle obstacle12[12] = {
        {-1,-1}, {0,-1}, {1,-1}, {2,-1}, {3,-1}, {4,-1},//外枠↓
        {4,3}, {3,3}, {2,3}, {1,3}, {0,3}, {-1,3}
    };
    std::vector<std::pair<int,int>> obstacles = {};
    //候補
    std::vector<std::pair<int,int>> candidates;//2次元
    std::vector<std::pair<int,int>> mandatory = { {1,0}, {2,2} };
    std::vector<std::pair<int,int>> uncollectedR1;
    std::vector<std::pair<int,int>> orderedUncollectedR1;
    std::vector<int> uncollectedR1_indices;
    std::vector<int> orderedR1_indices;
    int uncole1 = 0;
    int uncole2 = 0;
    int uncollect_box[12];
    int next_indices;
    

    void addCandidate(int x, int y);//candidates追加用関数
    void clearCandidates(); //クリア用関数
    void clearObstacles();
    void clearUncollect();
    void updateObstaclesFromSensors(bool obj[], int size);
    void addObstacleIfNew(int x, int y);//重複防止用
    void updateObstacles(bool obj[], int size);
    void addCandidateIfNew(int x, int y);
    void updateCandidates(bool obj[], int size);
    void addUncollectIfNew(int x, int y);
    void addUncollectindicesIfNew(int i);
    void wait_obj();
    int getCollectedCount();
    std::vector<std::pair<int,int>> directions; // 移動方向リスト
    std::vector<std::tuple<int,int,int>> directions3;
    std::vector<int> indices;
    // ゴール候補（複数）
    std::vector<std::pair<int,int>> goals = { {5,0}, {5,1}, {5,2} };
    // bool isGoal(int x, int y);
    bool isGoal(int x, int y,
                   const std::vector<std::pair<int,int>>& goals);
    //void AStar();//探索用関数
    bool gateOpen = false;

    double dist_plus_x = 0.0, dist_plus_y = 0.0;
    double tar_posi_x = 0.0, tar_posi_y = 0.0;
    double now_posi_x = 0.0, now_posi_y = 0.0;
    bool obj_true[12];
    bool pre_obj_true[12];
    int obj_true1[12];
    bool waitobj = false;
    bool empty_flag = false;
    bool empty_flag2 = false;
    
    int wait_id;
    bool obs_posi_true[12];//障害物がその場所にあったら1
    bool collect_posi_true[12];//回収物がその場所にあったら1を変えす．
    int obs_num[4] = {
        0,0,0,0
    };
    int collect_num[4] = {
        0,0,0,0
    };
    bool obj_max = false;
    bool R1KFS_posi[12];
    bool R2KFS_posi[12] = {
        0,0,0,
        0,0,0,
        0,0,0,
        0,0,0
    };
    
    bool Fake_posi[12];
    bool pre_R1KFS_posi[12];
    bool pre_R2KFS_posi[12];
    bool pre_Fake_posi[12];
    int Fake_posix = 0, Fake_posiy = 0;

int limit_collect_num = 0;
int collect_count = 0;
bool through_flag = false;

int cubePosi_x = 0;
int cubePosi_y = 0;
int dx_ = 0;
int dy_ = 0;
int nextX = 0;//次のcubePosi
int nextY = 0;
int nextZ = 0;
int nextX2 = 0, nextY2 = 0, nextZ2 = 0;
bool samePosi_flag = false;
bool samePosi_flag_Adjacent = false;
int current_x = 0, current_y = 0, current_z = 0;
int can_size = 0;
//ノード
// Node Open = {0, 0, 0.0, 0.0, 0.0, 0, 0};
// Node Close = {0, 0, 0.0, 0.0, 0.0, 0, 0};
Node Open = {0, 0, 0, 0.0, 0.0, 0.0, 0, 0, 0};
Node Close = {0, 0, 0, 0.0, 0.0, 0.0, 0, 0, 0};

//障害物候補
// Obstacle objePosi[15] = {
//     {3,0}, {3,1}, {3,2},
//     {2,0}, {2,1}, {2,2},
//     {1,0}, {1,1}, {1,2},
//     {0,0}, {0,1}, {0,2},
//     {-1,0}, {-1,1}, {-1,2}
// };

Obstacle objePosi2[12] = {
    {0,0}, {0,1}, {0,2},
    {1,0}, {1,1}, {1,2},
    {2,0}, {2,1}, {2,2},
    {3,0}, {3,1}, {3,2},
};

Obstacle objePosi3[12] = {// candidates用
    {0,2}, {0,1}, {0,0},
    {1,2}, {1,1}, {1,0},
    {2,2}, {2,1}, {2,0},
    {3,2}, {3,1}, {3,0},
};

int height_Posi[12] = {//段差の高さ数値
    1, 0, 1,
    0, 1, 2,
    1, 2, 1,
    0, 1, 0,
};

int heightMap[4][3]= {//段差の高さ数値
    {1, 0, 1},
    {0, 1, 2},
    {1, 2, 1},
    {0, 1, 0},
};

//マップの定義-------------------------
//0 = 何もない，1 = R1 , 2 = R2 , 3 = Fake
int mapData[6][5] = {//左上(0,0)
  {3,0,0,0,3},
  {3,2,1,0,3},
  {3,0,2,0,3},
  {3,2,3,2,3},
  {3,1,1,0,3},
  {3,0,0,0,3}
};
//高さデータ----------
int height[6][5] = {//左上(0,0)
    {0,0,0,0,0},
    {0,0,1,0,0},
    {0,1,2,1,0},
    {0,2,1,0,0},
    {0,1,0,1,0},
    {0,0,0,0,0}
};

const int COST_FLAT  = 10;   // 通常の移動コスト
const int COST_UP    = 25;   // 上り
const int COST_DOWN  = 15;   // 下り

Obstacle3 objePosi4[12] = {// candidates用 3次元
    {0,2,2}, {0,1,1}, {0,0,2},
    {1,2,1}, {1,1,2}, {1,0,3},
    {2,2,2}, {2,1,3}, {2,0,2},
    {3,2,1}, {3,1,2}, {3,0,1},
};

controllPoint forestPosi[15] = {
    {7.4,1.8}, {7.4,3.0}, {7.4,4.2},
    {6.2,1.8}, {6.2,3.0}, {6.2,4.2},
    {5.0,1.8}, {5.0,3.0}, {5.0,4.2},
    {3.8,1.8}, {3.8,3.0}, {3.8,4.2},
    {2.6,1.8}, {2.6,3.0}, {2.6,4.2}
};

controllPoint forestPosi2[15] = {
    {2.6,4.2}, {2.6,3.0}, {2.6,1.8},
    {3.8,4.2}, {3.8,3.0}, {3.8,1.8},
    {5.0,4.2}, {5.0,3.0}, {5.0,1.8},
    {6.2,4.2}, {6.2,3.0}, {6.2,1.8},
    {7.4,4.2}, {7.4,3.0}, {7.4,1.8}
};

controllPoint forestPosi5[18] = {
    {2.6,4.2}, {2.6,3.0}, {2.6,1.8},
    {3.8,4.2}, {3.8,3.0}, {3.8,1.8},
    {5.0,4.2}, {5.0,3.0}, {5.0,1.8},
    {6.2,4.2}, {6.2,3.0}, {6.2,1.8},
    {7.4,4.2}, {7.4,3.0}, {7.4,1.8},
    {8.6,4.2}, {8.6,3.0}, {8.6,1.8}
};

Obstacle objePosi5[18] = {// candidates用
    {-1,2}, {-1,1}, {-1,0},
    {0,2}, {0,1}, {0,0},
    {1,2}, {1,1}, {1,0},
    {2,2}, {2,1}, {2,0},
    {3,2}, {3,1}, {3,0},
    {4,2}, {4,1}, {4,0},
};

field forestXYMAX = {5, 3};//縦×横最大マス

int print_x, print_y;

    private:

};

#endif