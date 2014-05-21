PCL_ADD_EXAMPLE(pcl_example_remove_nan_from_point_cloud FILES example_remove_nan_from_point_cloud.cpp
                LINK_WITH pcl_common pcl_filters)

PCL_ADD_EXAMPLE(pcl_example_extract_indices FILES example_extract_indices.cpp
                LINK_WITH pcl_common pcl_filters)

PCL_ADD_EXAMPLE(pcl_example_median_filter FILES example_median_filter.cpp
                LINK_WITH pcl_common pcl_ocl_filters pcl_ocl_utils pcl_filters pcl_io)
