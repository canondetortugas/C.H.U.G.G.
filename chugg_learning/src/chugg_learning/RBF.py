"""Radial Basis Function Representation"""

from rlpy.Tools import perms
from rlpy.Representations.Representation import Representation
import numpy as np
import types
import logging

__copyright__ = "Copyright 2013, RLPy http://www.acl.mit.edu/RLPy"
__credits__ = ["Alborz Geramifard", "Robert H. Klein", "Christoph Dann",
               "William Dabney", "Jonathan P. How"]
__license__ = "BSD 3-Clause"
__author__ = "Alborz Geramifard"


class RBF(Representation):
    """ 
    Representation that uses a weighted sum of radial basis functions (RBFs)
    to reconstruct the value function.  Each RBF has a mean and variance 
    based on user-specified resolution_min, resolution_max, and grid_bins.
    See specification in __init__ below.
    
    
    """
    state_dimensions = None
    rbfs_mu = None  #: The mean of RBFs
    #: The std dev of the RBFs (uniformly selected between [0, dimension width]
    rbfs_sigma = None

    def __init__(self, domain, num_rbfs=None, state_dimensions=None,
                 const_feature=True, resolution_min=2., resolution_max=None,
                 seed=1, normalize=False, grid_bins=None, include_border=False):
        """
        :param domain: the :py:class`~rlpy.Domains.Domain.Domain` associated 
            with the value function we want to learn.
        :param num_rbfs: (Optional) Number of RBFs to use for fnctn
            approximation.  *THIS IS IGNORED* if grid_bins != None, and is 
            instead determined by the resolution.
        :param state_dimensions: (Optional) Allows user to select subset of 
            state dimensions for representation: ndarray.
        :param const_feature: Boolean, set true to allow for constant offset
        :param resolution_min: If ``grid_bins`` is specified, the standard 
            deviation sigma of each RBF is given by the average with 
            ``resolution_max``; otherwise it is selected uniform random in the 
            range with resolution_max.
        :param resolution_max: If ``grid_bins`` is specified, the standard 
            deviation sigma of each RBF is given by the average with 
            ``resolution_min``; otherwise it is selected uniform random in the 
            range with resolution_min.
        :param seed: To seed the random state generator when placing RBFs.
        :param normalize: (Boolean) If true, normalize returned feature 
            function values phi(s) so that sum( phi(s) ) = 1.
        :param grid_bins: ndarray, an int for each dimension, determines 
            discretization of each dimension.
        :param include_border: (Boolean) If true, adds an extra RBF to include 
            the domain boundaries.
        
        """

        if resolution_max is None:
            resolution_max = resolution_min

        if state_dimensions is not None:
            dims = len(state_dimensions)
        else:  # just consider all dimensions
            state_dimensions = range(domain.state_space_dims)
            dims = domain.state_space_dims

        if grid_bins is not None:
            # uniform grid of rbfs
            self.rbfs_mu, num_rbfs = self.uniformRBFs(
                grid_bins, include_border)
            self.rbfs_sigma = np.ones(
                (num_rbfs, dims)) * (resolution_max + resolution_min) / 2
        else:
            # uniformly scattered
            assert(num_rbfs is not None)
            self.rbfs_mu = np.zeros((num_rbfs, dims))
            self.rbfs_sigma = np.zeros((num_rbfs, dims))
            dim_widths = (domain.statespace_limits[state_dimensions, 1]
                          - domain.statespace_limits[state_dimensions, 0])
            rand_stream = np.random.RandomState(seed=seed)
            for i in xrange(num_rbfs):
                for d in state_dimensions:
                    self.rbfs_mu[i, d] = rand_stream.uniform(
                        domain.statespace_limits[d, 0],
                        domain.statespace_limits[d, 1])
                    self.rbfs_sigma[i,
                                    d] = rand_stream.uniform(
                        dim_widths[d] / resolution_max,
                        dim_widths[d] / resolution_min)
        self.const_feature = const_feature
        self.features_num = num_rbfs
        if const_feature:
            self.features_num += 1  # adds a constant 1 to each feature vector
        self.state_dimensions = state_dimensions
        self.normalize = normalize
        super(RBF, self).__init__(domain)

    def phi_nonTerminal(self, s):
        F_s = np.ones(self.features_num)
        if self.state_dimensions is not None:
            s = s[self.state_dimensions]

        exponent = np.sum(
            0.5 * ((s - self.rbfs_mu) / self.rbfs_sigma) ** 2,
            axis=1)
        if self.const_feature:
            F_s[:-1] = np.exp(-exponent)
        else:
            F_s[:] = np.exp(-exponent)

        if self.normalize and F_s.sum() != 0.:
            F_s /= F_s.sum()
        return F_s

    def uniformRBFs(self, bins_per_dimension, includeBorders=False):
        """
        :param bins_per_dimension: Determines the number of RBFs to place 
            uniformly in each dimension, see example below.
        :param includeBorders: (Boolean) If true, adds an extra RBF to include 
            the domain boundaries.
            
        Positions RBF Centers uniformly across the state space.\n
        Returns the centers as RBFs-by-dims matrix and number of rbfs. 
        Each row is a center of an RBF. \n
        Example: 2D domain where each dimension is in [0,3]
        with bins = [2,3], False => we get 1 center in the first dimension and 
        2 centers in the second dimension, hence the combination is:\n
        1.5    1 \n
        1.5    2 \n
        with parameter [2,3], True => we get 3 center in the first dimension 
        and 5 centers in the second dimension, hence the combination is: \n
        0      0 \n
        0      1 \n
        0      2 \n
        0      3 \n
        1.5    0 \n
        1.5    1 \n
        1.5    2 \n
        1.5    3 \n
        3      0 \n
        3      1 \n
        3      2 \n
        3      3 \n
        
        """
        domain = self.domain
        dims = domain.state_space_dims
        if includeBorders:
            rbfs_num = np.prod(bins_per_dimension[:] + 1)
        else:
            rbfs_num = np.prod(bins_per_dimension[:] - 1)
        all_centers = []
        for d in xrange(dims):
            centers = np.linspace(domain.statespace_limits[d, 0],
                                  domain.statespace_limits[d, 1],
                                  bins_per_dimension[d] + 1)
            if not includeBorders:
                centers = centers[1:-1]  # Exclude the beginning and ending
            all_centers.append(centers.tolist())
        # print all_centers
        # Find all pair combinations of them:
        result = perms(all_centers)
        # print result.shape
        return result, rbfs_num

    def featureType(self):
        return float

    def __getstate__(self):
        x = self.__dict__
	del x['logger']
	return x

    def __setstate__(self, d):
	d['logger'] = self.logger = logging.getLogger("rlpy.Representations." + self.__class__.__name__)
	self.dict = d
