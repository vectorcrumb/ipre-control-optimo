using DifferentialEquations
using Sundials
using Plots; pyplot()



function f(out, du, u, p, t)
    out[1] = -0.04u[1] + 1e4*u[2]*u[3] - du[1]
    out[2] = 0.04u[1] - 3e7*u[2]^2 - 1e4*u[2]*u[3] - du[2]
    out[3] = u[1] + u[2] + u[3]
end

u0 = [1.0, 0, 0]
du0 = [-0.04, 0.04, 0.0]
tspan = (0.0, 100000.0)
differential_vars = [true, true, false]

prob = DAEProblem(f, du0, u0, tspance, differential_vars=differential_vars)
sol = solve(prob, IDA())
plot(sol, xscale=:log10, tspan=(1e-6, 1e5), layout=(3, 1))