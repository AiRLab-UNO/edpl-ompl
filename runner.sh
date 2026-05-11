#!/usr/bin/env bash
# runner.sh — unified launcher for FIRM / FIRMCP / FIRMCP+STL demos.
#
# Subcommands
#   ./runner.sh stl    [yaml] [spec] [plan_seconds]   # STL benchmark (default)
#   ./runner.sh firm   [setup.xml]                    # original FIRM demo
#   ./runner.sh firmcp [setup.xml]                    # original FIRMCP demo
#   ./runner.sh build                                 # (re)build all targets
#   ./runner.sh analyze [run_dir]                     # summarise a run dir
#   ./runner.sh latest                                # analyse newest run dir
#
# `./runner.sh` with no args ≡ `./runner.sh stl`.
# Results land under Results/<run_kind>-<TIMESTAMP>/.

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$SCRIPT_DIR"

DEFAULT_YAML="examples/stl/bench_small.yaml"
DEFAULT_SPEC="examples/stl/bench_small.spec"
DEFAULT_SECONDS=15
DEFAULT_EXEC_STEPS=0   # 0 = roadmap-only smoke; >0 = run POMCP executor

# ---------- helpers --------------------------------------------------------

c_bold()   { printf '\033[1m%s\033[0m' "$*"; }
c_dim()    { printf '\033[2m%s\033[0m' "$*"; }
c_red()    { printf '\033[31m%s\033[0m' "$*"; }
c_green()  { printf '\033[32m%s\033[0m' "$*"; }
c_yellow() { printf '\033[33m%s\033[0m' "$*"; }

build_targets() {
    local targets=("$@")
    if [[ ${#targets[@]} -eq 0 ]]; then
        targets=(bsp-app-demo firmcp-demo firmcp-stl-demo)
    fi
    if [[ ! -d build ]]; then
        cmake -S . -B build >/dev/null
    fi
    cmake --build build --target "${targets[@]}" -- -j"$(nproc)"
}

ensure_binary() {
    local bin="$1" target="$2"
    if [[ ! -x "./$bin" ]]; then
        echo "$(c_yellow [build]) ./$bin missing — building $target"
        build_targets "$target"
    fi
}

print_run_summary() {
    local run_dir="$1"
    [[ -d "$run_dir" ]] || { echo "$(c_red [error]) not a directory: $run_dir"; return 1; }

    echo
    echo "$(c_bold "Results: ") $run_dir"

    if [[ -f "$run_dir/benchmark.csv" ]]; then
        echo
        echo "$(c_bold benchmark.csv)"
        column -s, -t < "$run_dir/benchmark.csv" | sed 's/^/  /'
    fi

    # Per-variant inspection
    local found_variant=0
    for vdir in "$run_dir"/*/; do
        [[ -d "$vdir" ]] || continue
        local variant
        variant=$(basename "$vdir")
        [[ "$variant" == "run-"* ]] && continue
        found_variant=1

        echo
        echo "$(c_bold "[$variant]")"
        if [[ -f "$vdir/summary.txt" ]]; then
            sed 's/^/  /' "$vdir/summary.txt"
        fi

        local sub
        sub=$(ls -d "$vdir"run-* 2>/dev/null | head -1 || true)
        if [[ -n "$sub" ]]; then
            local rm_xml db dij dp
            rm_xml=$(ls "$sub"/FIRMRoadMap-*.xml 2>/dev/null | tail -1 || true)
            db="$sub/results.db"
            dij="$sub/DijkstraSolveTime.txt"
            dp="$sub/DPSolveTime.txt"
            if [[ -n "$rm_xml" ]]; then
                local n_nodes n_edges
                n_nodes=$(grep -c '<node ' "$rm_xml" || true)
                n_edges=$(grep -c '<edge ' "$rm_xml" || true)
                echo "  roadmap_xml: $rm_xml"
                echo "  roadmap_nodes: $n_nodes"
                echo "  roadmap_edges: $n_edges"
            fi
            [[ -f "$dij" ]] && echo "  dijkstra: $(tr -d '\n' < "$dij")"
            [[ -f "$dp"  ]] && echo "  dp:       $(tr -d '\n' < "$dp")"
            if [[ -f "$db" ]] && command -v sqlite3 >/dev/null; then
                echo "  results.db tables:"
                while IFS= read -r tbl; do
                    local cnt
                    cnt=$(sqlite3 "$db" "SELECT count(*) FROM $tbl;" 2>/dev/null || echo "?")
                    printf "    %-18s %s rows\n" "$tbl" "$cnt"
                done < <(sqlite3 "$db" "SELECT name FROM sqlite_master WHERE type='table' ORDER BY name;")
                echo "  edge cost  (min/avg/max): $(sqlite3 "$db" "SELECT min(cost), round(avg(cost),2), max(cost) FROM roadmap_edges;" 2>/dev/null | tr '|' ' / ')"
                echo "  edge p_succ(min/avg/max): $(sqlite3 "$db" "SELECT round(min(success_prob),3), round(avg(success_prob),3), round(max(success_prob),3) FROM roadmap_edges;" 2>/dev/null | tr '|' ' / ')"

                # Execution telemetry — only populated when the executor ran (full mode)
                local n_vel n_cost
                n_vel=$(sqlite3 "$db" "SELECT count(*) FROM velocity;"     2>/dev/null || echo 0)
                n_cost=$(sqlite3 "$db" "SELECT count(*) FROM cost_history;" 2>/dev/null || echo 0)
                if [[ "${n_vel:-0}" -gt 0 ]]; then
                    echo "  exec velocity(min/avg/max): $(sqlite3 "$db" "SELECT round(min(velocity),3), round(avg(velocity),3), round(max(velocity),3) FROM velocity;" 2>/dev/null | tr '|' ' / ')"
                fi
                if [[ "${n_cost:-0}" -gt 0 ]]; then
                    echo "  exec final cost:           $(sqlite3 "$db" "SELECT round(cost,2) FROM cost_history ORDER BY timestep DESC LIMIT 1;" 2>/dev/null)"
                    echo "  exec final cost_cov:       $(sqlite3 "$db" "SELECT round(cost_cov,3) FROM cost_history ORDER BY timestep DESC LIMIT 1;" 2>/dev/null)"
                fi
            fi
        fi
    done

    if [[ $found_variant -eq 0 ]]; then
        echo "$(c_dim "  (no variant subdirs — single-run output)")"
    fi
}

latest_results_dir() {
    ls -td Results/firmcp_stl-* 2>/dev/null | head -1
}

# ---------- subcommands ----------------------------------------------------

cmd_build() {
    build_targets "$@"
}

cmd_firm() {
    local setup="${1:-SetupFiles/SetupFIRMCP.xml}"
    [[ -f "$setup" ]] || { echo "$(c_red [error]) setup not found: $setup"; exit 1; }
    ensure_binary bsp-app-demo bsp-app-demo
    echo "$(c_bold [firm]) ./bsp-app-demo $setup"
    ./bsp-app-demo "$setup"
}

cmd_firmcp() {
    local setup="${1:-SetupFiles/SetupFIRMCP.xml}"
    [[ -f "$setup" ]] || { echo "$(c_red [error]) setup not found: $setup"; exit 1; }
    ensure_binary firmcp-demo firmcp-demo
    echo "$(c_bold [firmcp]) ./firmcp-demo $setup"
    ./firmcp-demo "$setup"
}

cmd_stl() {
    local yaml="${1:-$DEFAULT_YAML}"
    local spec="${2:-$DEFAULT_SPEC}"
    local plan_seconds="${3:-$DEFAULT_SECONDS}"
    local max_exec_steps="${4:-$DEFAULT_EXEC_STEPS}"

    [[ -f "$yaml" ]] || { echo "$(c_red [error]) yaml not found: $yaml"; exit 1; }
    [[ -f "$spec" ]] || { echo "$(c_red [error]) spec not found: $spec"; exit 1; }

    ensure_binary firmcp-stl-demo firmcp-stl-demo

    echo "$(c_bold ============================================)"
    echo "$(c_bold ' yaml           ')$yaml"
    echo "$(c_bold ' spec           ')$spec"
    echo "$(c_bold ' plan_seconds   ')$plan_seconds"
    echo "$(c_bold ' max_exec_steps ')$max_exec_steps"
    echo "$(c_bold ' RSS cap (kB)   ')${FIRMCP_VMEM_KB:-4194304}"
    echo "$(c_bold ============================================)"

    # Hard memory cap so POMCP tree growth can't OOM the host.
    # Override with FIRMCP_VMEM_KB=<kB> before invocation. 4 GiB default.
    ( ulimit -v "${FIRMCP_VMEM_KB:-4194304}"
      exec ./firmcp-stl-demo "$yaml" "$spec" "$plan_seconds" "$max_exec_steps" )

    local run_dir
    run_dir=$(latest_results_dir || true)
    [[ -n "$run_dir" ]] && print_run_summary "$run_dir"
}

# Convenience: full benchmark = STL run with executor enabled.
cmd_full() {
    local yaml="${1:-$DEFAULT_YAML}"
    local spec="${2:-$DEFAULT_SPEC}"
    local plan_seconds="${3:-$DEFAULT_SECONDS}"
    local max_exec_steps="${4:-100}"   # sane default that exercises rollout
    cmd_stl "$yaml" "$spec" "$plan_seconds" "$max_exec_steps"
}

cmd_analyze() {
    local target="${1:-}"
    if [[ -z "$target" ]]; then
        target=$(latest_results_dir || true)
        [[ -n "$target" ]] || { echo "$(c_red [error]) no Results/firmcp_stl-* directories found"; exit 1; }
        echo "$(c_dim '(no path given — using latest)')"
    fi
    print_run_summary "$target"
}

cmd_latest() { cmd_analyze; }

cmd_help() {
    cat <<EOF
$(c_bold runner.sh) — FIRM / FIRMCP / FIRMCP+STL launcher

Subcommands:
  $(c_bold stl)    [yaml] [spec] [secs] [max_steps]  STL benchmark (default)
                                          max_steps=0 -> roadmap-only smoke
                                          max_steps>0 -> POMCP exec for N steps
                                          defaults: $DEFAULT_YAML
                                                    $DEFAULT_SPEC
                                                    ${DEFAULT_SECONDS}s plan / ${DEFAULT_EXEC_STEPS} steps
  $(c_bold full)   [yaml] [spec] [secs] [max_steps]  Same as 'stl' but max_steps=100
                                          (i.e. exercises the rollout policy)
  $(c_bold firm)   [setup.xml]                       original FIRM demo
  $(c_bold firmcp) [setup.xml]                       original FIRMCP demo
  $(c_bold build)  [target ...]                      (re)build (default: all three demos)
  $(c_bold analyze) [run_dir]                        summarise a Results/firmcp_stl-* run
  $(c_bold latest)                                   analyse the most recent STL run

Examples:
  ./runner.sh                                                # roadmap-only smoke
  ./runner.sh full                                           # full benchmark, 100 exec steps
  ./runner.sh stl examples/stl/bench_small.yaml examples/stl/bench_small.spec 30 200
  ./runner.sh firmcp SetupFiles/SetupFIRMCP.xml
  ./runner.sh analyze Results/firmcp_stl-20260510T173922
EOF
}

# ---------- dispatch -------------------------------------------------------

cmd="${1:-stl}"
shift || true
case "$cmd" in
    stl)        cmd_stl "$@" ;;
    full)       cmd_full "$@" ;;
    firm)       cmd_firm "$@" ;;
    firmcp)     cmd_firmcp "$@" ;;
    build)      cmd_build "$@" ;;
    analyze)    cmd_analyze "$@" ;;
    latest)     cmd_latest "$@" ;;
    help|-h|--help) cmd_help ;;
    *)
        echo "$(c_red [error]) unknown subcommand: $cmd"
        echo
        cmd_help
        exit 1
        ;;
esac
