import numpy as np
import random
from math import inf

class Particle:
    """
    Represents a particle of the Particle Swarm Optimization algorithm.
    """
    def __init__(self, lower_bound, upper_bound):
        """
        Creates a particle of the Particle Swarm Optimization algorithm.

        :param lower_bound: lower bound of the particle position.
        :type lower_bound: numpy array.
        :param upper_bound: upper bound of the particle position.
        :type upper_bound: numpy array.
        """
        # Todo: implement
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        self.delta = np.array(upper_bound - lower_bound)
        self.position = self.initial_particle_position()
        self.v = self.initial_particle_velocity()
        self.best_position = None
        self.best_value = -inf
 
    def initial_particle_position(self):
        """
        Generates the initial position of a particle within the defined bounds.

        Returns: np.ndarray
        """
        return np.array([np.random.uniform(self.lower_bound[i], self.upper_bound[i])
                          for i in range(len(self.lower_bound))])

    def initial_particle_velocity(self):
        """
        Initializes the particle's velocity within [-delta, delta].

        Returns: np.ndarray
        """
        return np.array([np.random.uniform(-self.delta[i], self.delta[i]) 
                         for i in range(len(self.delta))])
    
    def set_new_state(self, velocity):
        """
        Updates the particle's velocity and position based on the given velocity.
        Args:
            velocity (numpy.ndarray): The new velocity to be applied, which will be 
                                       clipped to the range [-delta, delta].
        Updates:
            self.v (numpy.ndarray): Clipped velocity.
            self.position (numpy.ndarray): Updated position, clipped to the range 
                                            [lower_bound, upper_bound].
        """
        self.v = np.clip(velocity, -self.delta, self.delta)

        new_position = self.position + self.v
        self.position = np.clip(new_position, self.lower_bound, self.upper_bound)

    def take_new_value(self, value):
        """
        Updates the particle's best known value and position if the given value is better.

        Args:
            value (float): The new value to compare with the current best value.
        """
        if self.best_value < value:
            self.best_value = value
            self.best_position = self.position.copy()


class ParticleSwarmOptimization:
    """
    Represents the Particle Swarm Optimization algorithm.
    Hyperparameters:
        inertia_weight: inertia weight.
        cognitive_parameter: cognitive parameter.
        social_parameter: social parameter.

    :param hyperparams: hyperparameters used by Particle Swarm Optimization.
    :type hyperparams: Params.
    :param lower_bound: lower bound of particle position.
    :type lower_bound: numpy array.
    :param upper_bound: upper bound of particle position.
    :type upper_bound: numpy array.
    """
    def __init__(self, hyperparams, lower_bound, upper_bound):
        # Todo: implement
        self.lower_bound = lower_bound
        self.upper_bound = upper_bound
        self.inertia_weight = hyperparams.inertia_weight
        self.cognitive_parameter = hyperparams.cognitive_parameter
        self.social_parameter = hyperparams.social_parameter
        self.num_particles = hyperparams.num_particles
        self.particles = self.initialize_particles()
        self.best_global_value = -inf
        self.best_global_position = None
        self.num_evaluation = 0

    def initialize_particles(self):
        """
        Initializes the particles for the swarm.

        Returns:
            list: A list of Particle objects initialized within the defined bounds.
        """
        return [Particle(self.lower_bound, self.upper_bound)
                for _ in range(self.num_particles)]

    def get_best_position(self):
        """
        Obtains the best position so far found by the algorithm.

        :return: the best position.
        :rtype: numpy array.
        """
        # Todo: implement
        return self.best_global_position  # Remove this line

    def get_best_value(self):
        """
        Obtains the value of the best position so far found by the algorithm.

        :return: value of the best position.
        :rtype: float.
        """
        return self.best_global_value  # Remove this line

    def get_position_to_evaluate(self):
        """
        Obtains a new position to evaluate.

        :return: position to evaluate.
        :rtype: numpy array.
        """
        # Todo: implement
        particle_index = self.num_evaluation % self.num_particles
        return self.particles[particle_index].position

    def advance_generation(self):
        """
        Advances the generation of particles. Auxiliary method to be used by notify_evaluation().
        """
        # Todo: implement
        for particle in self.particles: 
            rp = random.uniform(0.0, 1.0) 
            rg = random.uniform(0.0, 1.0)
            new_velocity = self.inertia_weight * particle.v + self.cognitive_parameter * rp * (particle.best_position - particle.position) + self.social_parameter * rg * (self.best_global_position - particle.position)
            particle.set_new_state(new_velocity)
     # Remove this line

    def notify_evaluation(self, value):
        """
        Notifies the algorithm that a particle position evaluation was completed.

        :param value: quality of the particle position.
        :type value: float.
        """
        # Todo: implement
        particle_index = self.num_evaluation % self.num_particles
        self.particles[particle_index].take_new_value(value)

        
        if value > self.best_global_value:
            self.best_global_value = value
            self.best_global_position = self.particles[particle_index].position.copy()

        self.num_evaluation += 1

        if  (((self.num_evaluation) % self.num_particles) == 0):
            self.advance_generation()
