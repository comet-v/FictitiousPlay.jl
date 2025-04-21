begin 
    using Pkg
    Pkg.activate(dirname(@__DIR__))
    Pkg.develop(path="C:\\Users\\lisve\\Documents\\POSGModels.jl/")
    Pkg.instantiate()
    using FictitiousPlay
    using POMDPTools
    using DiscreteValueIteration
    using LinearAlgebra
    #Pkg.activate(@__DIR__)
    using MarkovGames
    using POSGModels
    using POSGModels.StackedIntercept
    using POMDPs
    using Plots
    default(grid=false, framestyle=:box, fontfamily="Computer Modern", label="")
    using RecipesBase
    using StaticArrays
end

game = StackedInterceptMG(obstacles=Set{Coord}([Coord(4,5), Coord(6,3), Coord(10,9), Coord(9,3), Coord(11,9)]), SAMsites=Set{Coord}([Coord(5, 6), Coord(5, 10), Coord(16, 7), Coord(9, 9)]))
sol = FictitiousPlaySolver(
    verbose=true, 
    iter=10, 
    vi_solver = SparseValueIterationSolver(max_iterations=1000, belres=1e-3, verbose=true)
)
pol = solve(sol, game)
FictitiousPlay.clean_policy!(pol)

function gen_steps(game, pol::FictitiousPlayPolicy; max_steps=typemax(Int), s=rand(initialstate(game)))
    s_hist = []
    a_hist = []
    r_hist = Float64[]
    steps = Any[(;sp=s)]
    step = 0
    while step < max_steps && !isterminal(game, s)
        step += 1
        a_joint = FictitiousPlay.joint_action(pol, s)
        sp, r = @gen(:sp, :r)(game, s, a_joint)
        r = first(r)
        a = first(a_joint)
        push!(steps, (;a, sp, r))
        push!.((s_hist, a_hist, r_hist),(s, a, r))
        s = sp
    end
    return (;s=s_hist, a=a_hist, r=r_hist)
end

# can change initial state as desired
s0 = StackedInterceptState(SA[Coord(1,1), Coord(1,1)], Coord(5,5), false)
# e.g. put attackers and (10,7) and (11,6) and defender at (5,5)
# s0 = StackedInterceptState(SA[Coord(10,7), Coord(11,6)], Coord(5,5), false)
steps = gen_steps(game, pol; max_steps=30, s=s0)

anim = @animate for s in steps.s
    σ1,σ2 = FictitiousPlay.policies(pol, s)
    plot(game, s, σ1, σ2)
end

gif(anim, "stacked-intercept.gif", fps=5)


# plot an individual step
s = steps.s[1]
plot(game, s, FictitiousPlay.policies(pol, s)...)
