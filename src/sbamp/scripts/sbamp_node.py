#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from ..sbamp.ds_opt_py.lpv_opt.optimize_lpv_ds_from_data import optimize_lpv_ds_from_data
from ..sbamp.ds_opt_py.lpv_opt.lpv_ds import lpv_ds
    


from dataclasses import dataclass

@dataclass
class EstimationOptions:
    # Required parameters
    type: int            # 0=Non-parametric, 1=?, 2=?
    do_plots: bool       # Whether to generate visualizations
    
    # Optional parameters with defaults
    sub_sample: int = 1       # Subsampling factor (1=no subsampling)
    samplerIter: int = 0      # Number of sampler iterations
    l_sensitivity: float = 0  # Length-scale sensitivity factor
    estimate_l: bool = False  # Whether to auto-estimate length-scale
    length_scale: float = 1.0 # Manual length-scale (if estimate_l=False)



class SBAMPNode(Node):
    def __init__(self):
        super().__init__("sbamp_node")
        self.get_logger().info("Python sbamp_node has been started.")
        self.est_options = EstimationOptions(
            type=0,
            do_plots=True,
            sub_sample=2,
            estimate_l=True,
            l_sensitivity=2.0
        )


    def optimize_lpv_ds(self):
        A, b, P, gmm= optimize_lpv_ds_from_data(
            Data=None,  # Placeholder for actual data
            attractor=None,  # Placeholder for actual attractor
            ctr_type=0,  #0 for no P shaping, 1 for P shaping
            est_options=self.est_options,
            P=None,  # Placeholder for actual P matrix
            symm_constr=True  # Placeholder for symmetry constraint
        )
        return A, b, P, gmm

    def lpv_ds(self, x):
        A_g, b_g, P, ds_gmm = self.optimize_lpv_ds()
        x_dot = lpv_ds(x=x, ds_gmm=ds_gmm, A_g=A_g, b_g=b_g)
        return x_dot
    

def main(args=None):
    rclpy.init(args=args)
    node = SBAMPNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()