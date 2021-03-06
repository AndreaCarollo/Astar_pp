#include "PathPlanning.h"

// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~ Class functions ~~~~~~~~~~~~~~~~
// ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

PathPlanning::PathPlanning(ConfigReader *p)
{
    p->getValue("LOOK_AHEAD_DIST", max_dist);                 // Maximum ahead distance
    p->getValue("OBST_MIN_THRESH", low_threshold);            // Minimum height to be considered an obstacle
    p->getValue("OBST_MAX_THRESH", up_threshold);             // Maximum height to be considered an obstacle
    p->getValue("TARGET_THRESH", target_threshold);           // Maximum height to be considered an obstacle
    p->getValue("STOP_DISTANCE", distance_threshold);         // Distance from the target at which the control stops
    p->getValue("OBSTACLE_LEAF", (int &)obstacle_resolution); // Resolution with which the obstacles are searched
    p->getValue("GRID_SIZE", grid_size);                      // Size of the grid of the path planning
    p->getValue("PATH_PLANNING", path_planning);              // If path_planning is true, use the path planning algorithm
    p->getValue("OBST_LOOK_DOWN", look_down);                 // How much of the total point cloud is used to search the obstacle

    scale = grid_size / max_dist;           // distance scale from real [m] to grid
    robot = {grid_size - 1, grid_size / 2}; // position of the robot in the grid (index terms of row and column)

    offset_from_targer = target_threshold * scale; // An obstacle, to be considered, must be at least at target_threshold [mm] from the target

    refPnt = pcl::PointXYZ(0, 0, 0); // Reference point inizialization

    interface = Interface::getInstance(p); // Interface class initialization
    plane = Plane::getInstance(p);         // Plane class initialization
}

void PathPlanning::update(cv::Point *targetPoint2D, PntCld::Ptr PointCloud, cv::Size cvFrameSize)
{

    // update the plane with the current point cloud
    // find the reference point in 3D and transform using the plane transformation mtr
    // update the astar grid
    // Search and draw the obstacles into the interface and grid too
    // find the path (if the path_planning is true)
    // update the interface

    plane->update(PointCloud);

    // rs2_deproject_pixel_to_point(pt_cloud,this->intrinsic,pt_pixel,);
    refPnt = PointCloud->points[(targetPoint2D->y - 1) * cvFrameSize.width + targetPoint2D->x];
    refPnt = pcl::transformPoint(refPnt, plane->transf_mtx);

    target.row = robot.row - (refPnt.z) * scale;
    target.col = robot.col - (refPnt.x) * scale;

    distance_robot_target = std::sqrt(std::pow(refPnt.z, 2) + std::pow(refPnt.x, 2));

    interface->clean();

    grid = AStar_mtx(grid_size, std::vector<AStar_cell>(grid_size, {true, false, 1, nullptr, {0, 0}}));
    grid[robot.row][robot.col].visited = true;
    grid[robot.row][robot.col].cell = robot;

    obstacle_finding(PointCloud);

    if (distance_robot_target >= distance_threshold)
        if (path_planning)
            A_star();

    interface->update(this);
}

void PathPlanning::update(cv::Point3f *targetPoint3D, PntCld::Ptr PointCloud)
{

    // update the plane with the current point cloud
    // find the reference point in 3D and transform using the plane transformation mtr
    // update the astar grid
    // Search and draw the obstacles into the interface and grid too
    // find the path (if the path_planning is true)
    // update the interface

    plane->update(PointCloud);
    // // rs2_deproject_pixel_to_point(pt_cloud,this->intrinsic,pt_pixel,);
    refPnt = pcl::PointXYZ(targetPoint3D->x, targetPoint3D->y, targetPoint3D->z);
    refPnt = pcl::transformPoint(refPnt, plane->transf_mtx);

    target.row = robot.row - (refPnt.z) * scale;
    target.col = robot.col - (refPnt.x) * scale;

    distance_robot_target = std::sqrt(std::pow(refPnt.z, 2) + std::pow(refPnt.x, 2));

    interface->clean();

    grid = AStar_mtx(grid_size, std::vector<AStar_cell>(grid_size, {true, false, 1, nullptr, {0, 0}}));
    grid[robot.row][robot.col].visited = true;
    grid[robot.row][robot.col].cell = robot;

    obstacle_finding(PointCloud);

    if (distance_robot_target >= distance_threshold)
        if (path_planning)
            A_star();

    interface->update(this);
}

void PathPlanning::update(cv::Point *targetPoint2D, Stream *stream)
{

    // acquire the point cloud from the stream
    // project the input point (RGB RF) in the correct reference freme (IR RF)
    // update the plane with the current point cloud
    // find the reference point in 3D and transform using the plane transformation mtr
    // update the astar grid
    // Search and draw the obstacles into the interface and grid too
    // find the path (if the path_planning is true)
    // update the interface

    stream->PC_acq();

    stream->project_RGB2DEPTH(targetPoint2D);

    plane->update(stream->cloud);

    refPnt = pcl::transformPoint(stream->refPnt, plane->transf_mtx);

    target.row = robot.row - (refPnt.z) * scale;
    target.col = robot.col - (refPnt.x) * scale;

    distance_robot_target = std::sqrt(std::pow(refPnt.z, 2) + std::pow(refPnt.x, 2));

    interface->clean();

    grid = AStar_mtx(grid_size, std::vector<AStar_cell>(grid_size, {true, false, 1, nullptr, {0, 0}}));
    grid[robot.row][robot.col].visited = true;
    grid[robot.row][robot.col].cell = robot;

    if (distance_robot_target >= distance_threshold)
    {
        obstacle_finding(stream->cloud);

        if (path_planning)
            A_star();
    }

    interface->update(this);
}

void PathPlanning::obstacle_finding(PntCld::Ptr cloud)
{

    for (size_t i = look_down * cloud->size(); i < cloud->size(); i += obstacle_resolution)
    {
        tmpPnt = pcl::transformPoint(cloud->points[i], plane->transf_mtx);

        if (tmpPnt.y > low_threshold && tmpPnt.y < up_threshold)
        {
            int p_row = robot.row - (tmpPnt.z) * scale;
            int p_col = robot.col - (tmpPnt.x) * scale;

            if (p_row >= 0 && p_col >= 0 && p_row < grid_size && p_col < grid_size) // Obstacle inside the grid
            {

                // Obstacle not attached to the target
                if (!((p_row > target.row - offset_from_targer) && (p_row < target.row + offset_from_targer) &&
                      (p_col > target.col - offset_from_targer) && (p_col < target.col + offset_from_targer)))
                {
                    // Write the obstacle in the cell only the first time that we
                    // identify that cell as not free
                    if (grid[p_row][p_col].free)
                    {

                        interface->put_obstacle(p_col, p_row);

                        grid[p_row][p_col].cell.row = p_row;
                        grid[p_row][p_row].cell.col = p_col;
                        grid[p_row][p_col].free = false;
                    }
                }
            }
        }
    }
}

void PathPlanning::A_star()
{
    current = &grid[robot.row][robot.col]; // the starting point

    frontier.push(current);

    do
    {
        current = frontier.front();
        frontier.pop();
        neighbors(current);
    } while (frontier.size() != 0);
}

void PathPlanning::neighbors(AStar_cell *current)
{
    int r = current->cell.row, c = current->cell.col;

    for (int drow = -1; drow <= 1; drow++)
        for (int dcol = -1; dcol <= 1; dcol++)
            if (abs(drow) != abs(dcol))
                if ((r + drow >= 0) && (c + dcol >= 0) && (r + drow < grid_size) && (c + dcol < grid_size))
                    if (!(grid[r + drow][c + dcol].visited) && (grid[r + drow][c + dcol].free))
                    {
                        grid[r + drow][c + dcol].cell.col = c + dcol;
                        grid[r + drow][c + dcol].cell.row = r + drow;
                        grid[r + drow][c + dcol].came_from = current;
                        grid[r + drow][c + dcol].visited = true;
                        frontier.push(&grid[r + drow][c + dcol]);
                    }
}

void PathPlanning::extract_planned_path()
{
    struct AStar_cell *tmp_cell = &grid[target.row][target.col];
    cv::Point2i tmp_point;
    this->path_points.clear();

    do
    {
        int x_rect = tmp_cell->cell.col * this->interface->scale;
        int y_rect = tmp_cell->cell.row * this->interface->scale;
        tmp_point = cv::Point2i(x_rect, y_rect);
        this->path_points.push_back(tmp_point);
        tmp_cell = tmp_cell->came_from;

    } while (tmp_cell != NULL);
    reverse(path_points.begin(),path_points.end());
}

void PathPlanning::simplify_path()
{
    int start = 0;
    int end = 1;
    // int i = start;
    path_simplified.clear();
    std::cout << "new path, size " << path_points.size() << std::endl;
    // if (path_points.size() > 0)
    // {
    //     path_simplified.push_back(path_points[start]);
    //     do
    //     {
    //         for (i = start; i < path_points.size(); i++)
    //         {
    //             std::cout << i << std::endl;
    //             this->interface->put_simplified_path(path_points[start], path_points[i]);
    //             if (!check_intersection())
    //             {
    //                 path_simplified.push_back(path_points[i - 1]);
    //                 start = i - 1;
    //                 i = path_points.size() + 1;
    //             }
    //         }

    //     } while (i < path_points.size());
    //     // path_simplified.push_back(*path_points.end());
    // }

    if (path_points.size() > 0)
    {
        // reverse(path_points.begin(),path_points.end());
        path_simplified.push_back(path_points[start]);
        do
        {
            do
            {
                end++;
                this->interface->put_simplified_path(path_points[start], path_points[end]);

            } while (check_intersection() & end < path_points.size());
            start = end - 1;
            path_simplified.push_back(path_points[start]);

        } while (end < path_points.size());
        // path_simplified.push_back(*path_points.end());
    }
}

bool PathPlanning::check_intersection()
{
    cv::Mat m;
    interface->intersection_map_path.copyTo(m);
    bool is_collision = (std::find_if(m.begin<uchar>(), m.end<uchar>(),
                                      [](auto x) { return (x > 51); }) == m.end<uchar>());

    return is_collision;
}

void PathPlanning::smooth_path()
{
    this->extract_planned_path();
    // number_smooth
    for (int i = 0; i < 3; i++)
    {
        /* code */
        this->simplify_path();
        if (this->path_points.size() == this->path_simplified.size())
            break;
        this->path_points = this->path_simplified;
    }
}

void PathPlanning::path_wrt_world()
{
    path_simplified_wrt_world.clear();
    std::reverse(path_simplified.begin(), path_simplified.end());
    cv::Point2f tmp_point;
    for (int i = 0; i < path_simplified.size(); i++)
    {
        auto pp = cv::Point2f(path_simplified[i].x - path_simplified[0].x, -path_simplified[i].y + path_simplified[0].y);
        //- (cv::Point2f)path_simplified[i]/(float)this->interface->scale) ;// (float)this->scale;
        tmp_point = (pp / (float)this->interface->scale) / (float)this->scale;

        path_simplified_wrt_world.push_back(tmp_point);
    }
}