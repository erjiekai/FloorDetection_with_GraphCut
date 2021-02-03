
#include "flow_cut.hpp"
#include <vector>
#include "matrix_io.hpp"
#include "Eigen/Core"
#include "timer.hpp"

Eigen::MatrixXf RGB2BW(const Eigen::MatrixXf &rgb);
int main()
{
	float floor_threshold = 0.1;
	float obstacle_threshold = 0.7;
	float confirmed_floor_value = 0.01;
	float confirmed_obstacle_value = 1.0;

	MatrixIO mio;

	//read and process asgf image
	Eigen::MatrixXf asgf = mio.readFromFile("F:/SynologyDrive/MyDrive/Code/FloorDetection_with_GraphCut/data/asgf.txt");
	int image_rows = asgf.rows();
	int image_cols = asgf.cols();
	int image_pixels = image_rows * image_cols;

	for (int row = 0; row < image_rows; row++)
	{
		for (int col = 0; col < image_cols; col++)
		{
			if (asgf(row, col) == 1.0)
				asgf(row, col) = 0.0;
			else if (asgf(row, col) > obstacle_threshold)
				asgf(row, col) = obstacle_threshold;
		}
	}

	float asgf_max = obstacle_threshold;
	printf("asgf_max = %f\n", asgf_max);
	asgf = asgf.array() / asgf_max;

	for (int row = 0; row < image_rows; row++)
	{
		for (int col = 0; col < image_cols; col++)
		{
			if (asgf(row, col) == 0.0)
				asgf(row, col) = 0.5;
		}
	}

	for (int row = 0; row < image_rows; row++)
	{
		for (int col = 0; col < image_cols; col++)
		{
			if (asgf(row, col) <= floor_threshold)
				asgf(row, col) = confirmed_floor_value;
			else if (asgf(row, col) >= obstacle_threshold)
				asgf(row, col) = confirmed_obstacle_value;
		}
	}

	//read and process rgb image
	Eigen::MatrixXf rgb = mio.readFromFile("F:/SynologyDrive/MyDrive/Code/FloorDetection_with_GraphCut/data/camera_array.txt");
	Eigen::MatrixXf bw = RGB2BW(rgb);

	Timer timer;
	timer.start();

	// setting up the asgf graph
	Eigen::MatrixXf asgf_one_row_left = Eigen::MatrixXf::Zero(image_rows, image_cols);
	asgf_one_row_left.leftCols(image_cols - 1) = asgf.rightCols(image_cols - 1);

	Eigen::MatrixXf asgf_one_row_up = Eigen::MatrixXf::Zero(image_rows, image_cols);
	asgf_one_row_up.topRows(image_rows - 1) = asgf.bottomRows(image_rows - 1);

	Eigen::MatrixXf asgf_left_diff = (asgf - asgf_one_row_left).cwiseAbs();
	Eigen::MatrixXf asgf_up_diff = (asgf - asgf_one_row_up).cwiseAbs();

	// setting up the rgb graph
	Eigen::MatrixXf bw_one_row_left = Eigen::MatrixXf::Zero(image_rows, image_cols);
	bw_one_row_left.leftCols(image_cols - 1) = bw.rightCols(image_cols - 1);

	Eigen::MatrixXf bw_one_row_up = Eigen::MatrixXf::Zero(image_rows, image_cols);
	bw_one_row_up.topRows(image_rows - 1) = bw.bottomRows(image_rows - 1);

	Eigen::MatrixXf bw_left_diff = (bw - bw_one_row_left).cwiseAbs();
	Eigen::MatrixXf bw_up_diff = (bw - bw_one_row_up).cwiseAbs();

	// initialize FlowCut class
	int num_vertices = image_pixels + 2;																			 // 2 additional vertex for source and sink
	int num_edges = image_rows + image_cols - 1 - 1 + (image_rows * image_cols - (image_rows + image_cols - 1)) * 2; // number of edges interconnecting pixels
	num_edges = num_edges + image_pixels * 2;																		 // number of edges connecting source to pixels and pixels to sink
	FlowCut fc(num_vertices);

	//calculate probabilities
	Eigen::MatrixXf source2pixel = asgf.array();			  //source to pixel edges. coefficient wise multiplication
	Eigen::MatrixXf pixel2sink = 1.0 - source2pixel.array();  //pixel to sink edges
	Eigen::MatrixXf left_diff = 1.0 - (bw_left_diff.array()); //*asgf_left_diff.array()
	Eigen::MatrixXf up_diff = 1.0 - (bw_up_diff.array());	  //*asgf_up_diff.array()

	// edge for interconnecting pixels
	int edge_count = 0;
	for (int row = 0; row < image_rows; row++)
	{
		for (int col = 0; col < image_cols; col++)
		{
			// src to pixels
			{
				int src = 0;
				int dst = row * image_cols + col + 1;
				fc.add_edge(src, dst, int(1000.0 * source2pixel(row, col)));
				edge_count++;
				// printf("src=%d dst=%d capacity=%d\n", src, dst, int(1000.0 * source2pixel(row, col)));
				// std::cout << "Edge Count1 = " << edge_count << "/" << num_edges << std::endl;
			}

			// pixels to sink
			{
				int src = row * image_cols + col + 1;
				int dst = image_pixels + 1;
				if (source2pixel(row, col) <= floor_threshold || source2pixel(row, col) > obstacle_threshold)
				{
					fc.add_edge(src, dst, int(1000.0 * pixel2sink(row, col)));
					edge_count++;
					// printf("src=%d dst=%d capacity=%d\n", src, dst, int(1000.0 * pixel2sink(row, col)));
				}
				// std::cout << "Edge Count2 = " << edge_count << "/" << num_edges << std::endl;
			}

			// left minus right difference
			if (col != (image_cols - 1))
			{
				int src = row * image_cols + col + 1;
				int dst = src + 1;
				fc.add_edge(src, dst, int(1000.0 * left_diff(row, col)));
				edge_count++;
				// printf("src=%d dst=%d capacity=%d\n", src, dst, int(1000.0 * left_diff(row, col)));
				// std::cout << "Edge Count3 = " << edge_count << "/" << num_edges << std::endl;
			}

			// up minus down difference
			if (row != (image_rows - 1))
			{
				int src = row * image_cols + col + 1;
				int dst = src + image_cols;
				fc.add_edge(src, dst, int(1000.0 * up_diff(row, col)));
				edge_count++;
				// printf("src=%d dst=%d capacity=%d\n", src, dst, int(1000.0 * up_diff(row, col)));
				// std::cout << "Edge Count4 = " << edge_count << "/" << num_edges << std::endl;
			}
		}
	}

	// std::cout << "Edge Count = " << edge_count << "/" << num_edges << std::endl;
	// if (edge_count != num_edges)
	// {
	// 	perror("Counted edges do not equal to declared edges!\n");
	// }

	printf("Max flow between vertex %d and %d = %d\n", 0, num_vertices - 1, fc.max_flow(0, num_vertices - 1));

	vector<int> saturated_vertices = fc.saturated_vertices();
	printf("Flow Cut runtime = %f ms\n", timer.getInterval());
	printf("Number of Saturated Vertices = %d\n", saturated_vertices.size());
	Eigen::MatrixXf saturated_verices_matrix = Eigen::MatrixXf::Zero(image_rows, image_cols);
	for (int i = 0; i < saturated_vertices.size(); i++)
	{
		// printf("%d  ", saturated_vertices[i]);
		int row = (saturated_vertices[i] - 1) / image_cols;
		int col = (saturated_vertices[i] - 1) % image_cols;
		if (row < image_rows && col < image_cols && row > -1 && col > -1)
		{
			saturated_verices_matrix(row, col) = 1;
		}
	}
	mio.writeToFile("F:/SynologyDrive/MyDrive/Code/FloorDetection_with_GraphCut/data/saturated_vertices.txt", saturated_verices_matrix);
	mio.writeToFile("F:/SynologyDrive/MyDrive/Code/FloorDetection_with_GraphCut/data/asgf_left_diff.txt", asgf_left_diff);
	mio.writeToFile("F:/SynologyDrive/MyDrive/Code/FloorDetection_with_GraphCut/data/asgf_up_diff.txt", asgf_up_diff);
	mio.writeToFile("F:/SynologyDrive/MyDrive/Code/FloorDetection_with_GraphCut/data/bw.txt", bw);
	mio.writeToFile("F:/SynologyDrive/MyDrive/Code/FloorDetection_with_GraphCut/data/asgf_processed.txt", asgf);
	mio.writeToFile("F:/SynologyDrive/MyDrive/Code/FloorDetection_with_GraphCut/data/bw_left_diff.txt", bw_left_diff);
	mio.writeToFile("F:/SynologyDrive/MyDrive/Code/FloorDetection_with_GraphCut/data/bw_up_diff.txt", bw_up_diff);
	mio.writeToFile("F:/SynologyDrive/MyDrive/Code/FloorDetection_with_GraphCut/data/left_diff.txt", left_diff);
	mio.writeToFile("F:/SynologyDrive/MyDrive/Code/FloorDetection_with_GraphCut/data/up_diff.txt", up_diff);
	printf("End\n");

	// int num_vertex, num_edges;
	// printf("Number of vertex: ");
	// scanf_s("%d", &num_vertex);
	// printf("Number of edges: ");
	// scanf_s("%d", &num_edges);

	// FlowCut g(num_vertex);
	// for (int i = 0; i < num_edges; ++i)
	// {
	// 	int u, v, w; //u = source, v = destination, w = capacity
	// 	printf("Source, Dst, Capacity: ");
	// 	int numret = scanf("%d %d %d", &u, &v, &w);

	// 	g.add_edge(u, v, w);
	// }
	// printf("Max flow between vertex %d and %d = %lld\n", 0, num_vertex - 1, g.max_flow(0, num_vertex - 1));

	// vector<int> saturated_vertices = g.saturated_vertices();
	// printf("Saturated Vertices = ");
	// for (int i = 0; i < saturated_vertices.size(); i++)
	// 	printf("%d\t", saturated_vertices[i]);
}

/** @brief Convert RGB image to Black & White
 *
 * A weighted average of the RGB image is used to convert RGB to BW
 * New grayscale image = ( (0.3 * R) + (0.59 * G) + (0.11 * B) ).
 * 
 * @note https://www.tutorialspoint.com/dip/grayscale_to_rgb_conversion.htm#:~:text=Average%20method%20is%20the%20most,Its%20done%20in%20this%20way.
 *
 * @param[in] rgb the 2D matrix with rgb information. Each pixel is stored as 3 element: [[x1_red][x1_green][x1_blue] [x2_red][x2_green][x2_blue] ...]
 *
 * @return returns the bw matrix
 */
Eigen::MatrixXf RGB2BW(const Eigen::MatrixXf &rgb)
{
	int image_rows = rgb.rows();
	int image_cols = rgb.cols() / 3;
	int image_pixels = image_rows * image_cols;

	Eigen::MatrixXf result = Eigen::MatrixXf::Zero(image_rows, image_cols);

	for (int row = 0; row < image_rows; row++)
	{
		for (int col = 0; col < image_cols; col++)
		{
			result(row, col) = (0.3 * rgb(row, col * 3) + 0.11 * rgb(row, col * 3 + 1) + 0.59 * rgb(row, col * 3 + 2)) / 1000.0;
		}
	}

	return result;
}