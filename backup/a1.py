from a1_sim import A1

dt = 0.001
real_time_factor = 1.0
if __name__ == "__main__" :
    a1 = A1("a1/a1.urdf", dt, real_time_factor)
    while True:
        a1.simulation_step()