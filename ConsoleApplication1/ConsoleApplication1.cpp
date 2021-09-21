#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include "PathFinder.h"
#include <string>
#include <chrono>
using namespace PathFinder;

int main()
{
    Json::Value root;
    std::ifstream ifs;
    ifs.open("tst.json");

    Json::CharReaderBuilder builder;
    builder["collectComments"] = true;
    JSONCPP_STRING errs;
    if (!parseFromStream(builder, ifs, &root, &errs)) {
        std::cout << errs << std::endl;
        return EXIT_FAILURE;
    }
    MapGrid map(root);
    AStar as(map);
    CVector start(0, 0);
    CVector goal(3, 0);
    auto startTime = std::chrono::high_resolution_clock::now();
    std::list<GridNode> path = as.FindPath(start,goal);
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end - startTime);
    std::cout << "Execution took: " << duration.count() << " us" << std::endl;
    Path exPath1(start, goal, path);
    std::ofstream file;
    file.open("Test.json");
    Json::StreamWriterBuilder writeBuilder;
    const std::unique_ptr<Json::StreamWriter> writer(writeBuilder.newStreamWriter());
    std::string fiData = exPath1.GetFileData();
    file << fiData << std::endl;
    file.close();
    

    //Options
    /*
        Load a New Map
        Select a new start
        Select a goal
        Plan path
    */

    return 0;
}