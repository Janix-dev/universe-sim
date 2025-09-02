// Player's Own Universe Simulator — single-file C (SDL2)
// Build (Linux/macOS):
//   gcc universe_sim.c -o universe -O2 -Wall -Wextra -std=c11 -lSDL2 -lm
// On Debian/Ubuntu: sudo apt-get install libsdl2-dev
// On Fedora: sudo dnf install SDL2-devel
// On macOS (Homebrew): brew install sdl2 && (use clang instead of gcc)
//
// Controls (also printed to stdout):
//  • Mouse Wheel: Zoom in/out
//  • Right Drag: Pan camera
//  • Left Click: Spawn a planet (drag while held to set initial velocity)
//  • Right Click: Spawn a star (heavier)
//  • M: Toggle mass-visual size vs physical size
//  • T/G: Decrease/Increase time scale (simulation speed)
//  • [:]/;: Decrease/Increase gravity constant
//  • F: Toggle trails
//  • P: Pause/Resume
//  • R: Reset (clear all bodies)
//  • N: Procedurally generate a spiral galaxy
//  • S: Save current universe to universe.save
//  • L: Load universe from universe.save
//  • H: Toggle help overlay
//  • Esc/Q: Quit
//
// Notes:
//  • Physics is a simple N-body O(N^2) solver with softening.
//  • Units are arbitrary; tuned for stability and visual interest.
//  • Designed to compile with GCC and only depends on SDL2 and libm.
//
// © 2025 — Public Domain / CC0. Do whatever you like.

#include <SDL2/SDL.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define MAX_BODIES 5000
#define TRAIL_LEN 64

static const int WIN_W = 1280;
static const int WIN_H = 800;

// ------------------- Types -------------------

typedef struct { double x, y; } Vec2;

typedef struct {
    Vec2 pos, vel, acc;
    double mass;
    double radius; // drawing radius (in world units)
    Uint8 r, g, b, a;
    int isStatic;  // if 1, body doesn't move
    int alive;
    Vec2 trail[TRAIL_LEN];
    int trail_idx;
} Body;

typedef struct {
    Body bodies[MAX_BODIES];
    int count;
} Universe;

typedef struct {
    double cx, cy;  // camera center in world coordinates
    double zoom;    // pixels per world unit
} Camera;

// ------------------- Globals -------------------

static Universe U = {0};
static Camera C = { .cx = 0, .cy = 0, .zoom = 1.0 };

static int running = 1;
static int paused = 0;
static int show_trails = 1;
static int help_overlay = 1;
static int size_by_mass = 1;

static double G_CONST = 50.0;    // gravitational constant
static double TIME_SCALE = 1.0;  // delta-time multiplier

// ------------------- Utility -------------------

static inline double clampd(double v, double lo, double hi){ return v < lo ? lo : (v > hi ? hi : v); }
static inline double len2(Vec2 v){ return v.x*v.x + v.y*v.y; }
static inline double len(Vec2 v){ return sqrt(len2(v)); }
static inline Vec2 add(Vec2 a, Vec2 b){ return (Vec2){a.x+b.x, a.y+b.y}; }
static inline Vec2 sub(Vec2 a, Vec2 b){ return (Vec2){a.x-b.x, a.y-b.y}; }
static inline Vec2 scale(Vec2 a, double s){ return (Vec2){a.x*s, a.y*s}; }

static Uint32 rng_u32(void){
    static Uint32 x=123456789, y=362436069, z=521288629, w=88675123;
    Uint32 t = x ^ (x << 11);
    x = y; y = z; z = w;
    return w = w ^ (w >> 19) ^ (t ^ (t >> 8));
}
static double frand(void){ return (rng_u32() / (double)UINT32_MAX); }

// World <-> Screen transforms
static Vec2 world_to_screen(Vec2 p){
    double sx = (p.x - C.cx) * C.zoom + WIN_W/2.0;
    double sy = (p.y - C.cy) * C.zoom + WIN_H/2.0;
    return (Vec2){ sx, sy };
}
static Vec2 screen_to_world(Vec2 s){
    double wx = (s.x - WIN_W/2.0)/C.zoom + C.cx;
    double wy = (s.y - WIN_H/2.0)/C.zoom + C.cy;
    return (Vec2){ wx, wy };
}

// ------------------- Bodies -------------------

static Body *add_body(Vec2 pos, Vec2 vel, double mass, double radius, SDL_Color col, int isStatic){
    if (U.count >= MAX_BODIES) return NULL;
    Body *b = &U.bodies[U.count++];
    *b = (Body){0};
    b->pos = pos; b->vel = vel; b->mass = mass; b->radius = radius;
    b->r = col.r; b->g = col.g; b->b = col.b; b->a = 255;
    b->isStatic = isStatic; b->alive = 1; b->trail_idx = 0;
    for(int i=0;i<TRAIL_LEN;i++) b->trail[i]=pos;
    return b;
}

static void clear_universe(void){ U.count = 0; }

// Simple collision merge: inelastic merge if overlapping
static void handle_collisions(void){
    for(int i=0;i<U.count;i++){
        Body *a = &U.bodies[i]; if(!a->alive) continue;
        for(int j=i+1;j<U.count;j++){
            Body *b = &U.bodies[j]; if(!b->alive) continue;
            Vec2 d = sub(b->pos, a->pos);
            double dist = len(d);
            double rr = a->radius + b->radius;
            if (dist > 0 && dist < rr){
                // Merge smaller into larger
                Body *big=a, *sm=b;
                if (b->mass > a->mass){ big=b; sm=a; }
                double total = big->mass + sm->mass;
                // momentum conservation
                Vec2 v = scale(add(scale(big->vel, big->mass), scale(sm->vel, sm->mass)), 1.0/total);
                big->vel = v;
                big->mass = total;
                big->radius = pow(big->radius*big->radius*big->radius + sm->radius*sm->radius*sm->radius, 1.0/3.0);
                // color blend by mass
                double wb = big->mass/total, ws = sm->mass/total;
                big->r = (Uint8)clampd(big->r*wb + sm->r*ws, 0, 255);
                big->g = (Uint8)clampd(big->g*wb + sm->g*ws, 0, 255);
                big->b = (Uint8)clampd(big->b*wb + sm->b*ws, 0, 255);
                sm->alive = 0;
            }
        }
    }
    // Compact array
    int k=0; for(int i=0;i<U.count;i++){ if(U.bodies[i].alive) U.bodies[k++]=U.bodies[i]; }
    U.count = k;
}

// ------------------- Physics -------------------

static void integrate(double dt){
    // Reset accelerations
    for(int i=0;i<U.count;i++){ U.bodies[i].acc = (Vec2){0,0}; }

    // Pairwise gravity with softening to avoid singularity
    const double eps2 = 1.0; // softening^2
    for(int i=0;i<U.count;i++){
        Body *a = &U.bodies[i]; if(!a->alive) continue;
        for(int j=i+1;j<U.count;j++){
            Body *b = &U.bodies[j]; if(!b->alive) continue;
            Vec2 d = sub(b->pos, a->pos);
            double r2 = len2(d) + eps2;
            double invr = 1.0 / sqrt(r2);
            double invr3 = invr*invr*invr;
            double f = G_CONST * a->mass * b->mass * invr3;
            Vec2 acc_a = scale(d, f / a->mass);
            Vec2 acc_b = scale(d, -f / b->mass);
            if (!a->isStatic) a->acc = add(a->acc, acc_a);
            if (!b->isStatic) b->acc = add(b->acc, acc_b);
        }
    }

    // Integrate velocities and positions
    for(int i=0;i<U.count;i++){
        Body *b = &U.bodies[i];
        if (!b->isStatic){
            b->vel = add(b->vel, scale(b->acc, dt));
            b->pos = add(b->pos, scale(b->vel, dt));
        }
        // Trails
        if (show_trails){
            b->trail[b->trail_idx] = b->pos;
            b->trail_idx = (b->trail_idx + 1) % TRAIL_LEN;
        }
    }

    handle_collisions();
}

// ------------------- Rendering -------------------

static void draw_filled_circle(SDL_Renderer *R, int cx, int cy, int rad){
    for (int dy = -rad; dy <= rad; dy++){
        int dx = (int)sqrt((double)(rad*rad - dy*dy));
        SDL_RenderDrawLine(R, cx - dx, cy + dy, cx + dx, cy + dy);
    }
}

static void render(SDL_Renderer *R){
    // Clear background (deep space gray)
    SDL_SetRenderDrawColor(R, 6, 8, 15, 255);
    SDL_RenderClear(R);

    // Draw trails
    if (show_trails){
        SDL_SetRenderDrawBlendMode(R, SDL_BLENDMODE_BLEND);
        for(int i=0;i<U.count;i++){
            Body *b = &U.bodies[i];
            SDL_SetRenderDrawColor(R, b->r, b->g, b->b, 50);
            for(int k=0;k<TRAIL_LEN-1;k++){
                int idx0 = (b->trail_idx + k) % TRAIL_LEN;
                int idx1 = (b->trail_idx + k + 1) % TRAIL_LEN;
                Vec2 s0 = world_to_screen(b->trail[idx0]);
                Vec2 s1 = world_to_screen(b->trail[idx1]);
                SDL_RenderDrawLine(R, (int)s0.x, (int)s0.y, (int)s1.x, (int)s1.y);
            }
        }
    }

    // Draw bodies
    SDL_SetRenderDrawBlendMode(R, SDL_BLENDMODE_BLEND);
    for(int i=0;i<U.count;i++){
        Body *b = &U.bodies[i];
        Vec2 s = world_to_screen(b->pos);
        int rad_px = (int)fmax(1.0, (size_by_mass ? (pow(b->mass, 1.0/3.0)) : b->radius) * C.zoom);
        SDL_SetRenderDrawColor(R, b->r, b->g, b->b, 255);
        draw_filled_circle(R, (int)s.x, (int)s.y, rad_px);
        // tiny glow
        SDL_SetRenderDrawColor(R, b->r, b->g, b->b, 80);
        draw_filled_circle(R, (int)s.x, (int)s.y, rad_px+3);
    }

    // Minimal HUD (rectangles only to avoid extra deps)
    if (help_overlay){
        SDL_Rect box = { 10, 10, 520, 210 };
        SDL_SetRenderDrawColor(R, 20, 24, 36, 200);
        SDL_RenderFillRect(R, &box);
        SDL_SetRenderDrawColor(R, 100, 150, 255, 220);
        SDL_RenderDrawRect(R, &box);
        // We can't render text without SDL_ttf; draw illustrative tick marks
        // and rely on stdout for the full help. The box simply indicates help is on.
    }

    SDL_RenderPresent(R);
}

// ------------------- I/O: Save/Load -------------------

static int save_universe(const char *path){
    FILE *fp = fopen(path, "w"); if(!fp) return 0;
    fprintf(fp, "# universe save file\n");
    fprintf(fp, "G %f\nT %f\n", G_CONST, TIME_SCALE);
    fprintf(fp, "C %f %f %f\n", C.cx, C.cy, C.zoom);
    fprintf(fp, "N %d\n", U.count);
    for(int i=0;i<U.count;i++){
        Body *b=&U.bodies[i];
        fprintf(fp, "B %d %f %f %f %f %f %f %f %u %u %u %u %d\n", b->alive,
                b->pos.x, b->pos.y, b->vel.x, b->vel.y, b->mass, b->radius,
                0.0, b->r,b->g,b->b,b->a,b->isStatic);
    }
    fclose(fp);
    return 1;
}

static int load_universe(const char *path){
    FILE *fp = fopen(path, "r"); if(!fp) return 0;
    clear_universe();
    char tag[8];
    while (fscanf(fp, "%7s", tag)==1){
        if (tag[0]=='#'){ // comment line
            int c; while((c=fgetc(fp))!='\n' && c!=EOF){}
        } else if (strcmp(tag,"G")==0){ fscanf(fp, "%lf", &G_CONST); }
        else if (strcmp(tag,"T")==0){ fscanf(fp, "%lf", &TIME_SCALE); }
        else if (strcmp(tag,"C")==0){ fscanf(fp, "%lf %lf %lf", &C.cx, &C.cy, &C.zoom); }
        else if (strcmp(tag,"N")==0){ int n; fscanf(fp, "%d", &n); }
        else if (strcmp(tag,"B")==0){
            int alive,isStatic; double px,py,vx,vy,mass,rad,tmp; unsigned r,g,b,a;
            fscanf(fp, "%d %lf %lf %lf %lf %lf %lf %lf %u %u %u %u %d",
                   &alive,&px,&py,&vx,&vy,&mass,&rad,&tmp,&r,&g,&b,&a,&isStatic);
            if (alive){
                SDL_Color col={(Uint8)r,(Uint8)g,(Uint8)b,(Uint8)a};
                add_body((Vec2){px,py}, (Vec2){vx,vy}, mass, rad, col, isStatic);
            }
        } else {
            // skip rest of line
            int c; while((c=fgetc(fp))!='\n' && c!=EOF){}
        }
    }
    fclose(fp);
    return 1;
}

// ------------------- Galaxy Generator -------------------

static void spawn_spiral_galaxy(Vec2 center, int arms, int stars, double core_mass){
    // Central massive body
    SDL_Color corec = {255, 230, 180, 255};
    add_body(center, (Vec2){0,0}, core_mass, 6.0, corec, 0);

    for(int i=0;i<stars;i++){
        double arm = (double)(i % arms);
        double t = (double)i / stars; // 0..1
        double r = 10 + pow(t, 0.8) * 450.0 * (0.7 + 0.6*frand());
        double theta = arm * (2*M_PI/arms) + 4.5 * log(1 + r*0.02) + (frand()-0.5)*0.3;
        Vec2 p = { center.x + r*cos(theta), center.y + r*sin(theta) };
        // Circular orbit velocity around core (approx)
        double v = sqrt(G_CONST * core_mass / fmax(30.0, r));
        Vec2 tang = { -sin(theta), cos(theta) };
        Vec2 vel = scale(tang, v * (0.9 + 0.2*frand()));
        double mass = 0.5 + 2.0*frand();
        double rad = 1.0 + 1.5*frand();
        SDL_Color col = { (Uint8)(150+105*frand()), (Uint8)(150+105*frand()), (Uint8)(150+105*frand()), 255 };
        add_body(p, vel, mass, rad, col, 0);
    }
}

// ------------------- Input -------------------

typedef struct { int active; Vec2 start_world; } Drag;
static Drag drag = {0};

static void print_help_once(void){
    static int printed=0; if(printed) return; printed=1;
    printf("\n=== Player's Universe Simulator (C/SDL2) ===\n");
    printf("Mouse Wheel: Zoom | Right Drag: Pan | Left Click+Drag: Launch planet | Right Click: Spawn star\n");
    printf("M: size by mass | F: trails | P: pause | R: reset | N: new galaxy | S/L: save/load | H: toggle help\n");
    printf("G/; : gravity +/- | T/Y : time scale -/+ | Esc/Q: quit\n\n");
}

static void handle_event(SDL_Event *e, SDL_Window *W){
    if (e->type == SDL_QUIT){ running = 0; }
    else if (e->type == SDL_MOUSEWHEEL){
        double before_zoom = C.zoom;
        if (e->wheel.y > 0) C.zoom *= 1.1; else if (e->wheel.y < 0) C.zoom /= 1.1;
        C.zoom = clampd(C.zoom, 0.05, 10.0);
        // Zoom at mouse point
        int mx, my; SDL_GetMouseState(&mx, &my);
        Vec2 world_before = screen_to_world((Vec2){mx, my});
        Vec2 world_after  = screen_to_world((Vec2){mx, my});
        C.cx += world_before.x - world_after.x; C.cy += world_before.y - world_after.y;
    }
    else if (e->type == SDL_MOUSEBUTTONDOWN){
        int mx=e->button.x, my=e->button.y; Vec2 w = screen_to_world((Vec2){mx,my});
        if (e->button.button == SDL_BUTTON_RIGHT){
            // start panning
            drag.active = 2; // 2=pan
            drag.start_world = (Vec2){(double)mx, (double)my};
        } else if (e->button.button == SDL_BUTTON_LEFT){
            drag.active = 1; // 1=launch
            drag.start_world = w;
        }
    }
    else if (e->type == SDL_MOUSEBUTTONUP){
        int mx=e->button.x, my=e->button.y; Vec2 w = screen_to_world((Vec2){mx,my});
        if (drag.active == 1 && e->button.button == SDL_BUTTON_LEFT){
            Vec2 dv = sub(w, drag.start_world);
            Vec2 v = scale(dv, 0.5); // scale to taste
            double mass = 1.0 + 2.0*frand();
            double rad = 2.0 + 2.0*frand();
            SDL_Color col = { (Uint8)(100+155*frand()), (Uint8)(120+135*frand()), (Uint8)(150+105*frand()), 255 };
            add_body(drag.start_world, v, mass, rad, col, 0);
            drag.active = 0;
        }
        else if (drag.active == 2 && e->button.button == SDL_BUTTON_RIGHT){
            // right click without movement: spawn star
            int dx = (int)(mx - drag.start_world.x);
            int dy = (int)(my - drag.start_world.y);
            if (abs(dx)+abs(dy) < 3){
                double mass = 200.0 + 200.0*frand();
                double rad = 6.0 + 3.0*frand();
                SDL_Color col = { 255, (Uint8)(200+55*frand()), (Uint8)(80+60*frand()), 255 };
                add_body(w, (Vec2){0,0}, mass, rad, col, 0);
            }
            drag.active = 0;
        }
    }
    else if (e->type == SDL_MOUSEMOTION){
        if (drag.active == 2){ // panning
            int mx=e->motion.x, my=e->motion.y;
            double dx = (mx - drag.start_world.x)/C.zoom;
            double dy = (my - drag.start_world.y)/C.zoom;
            C.cx -= dx; C.cy -= dy;
            drag.start_world = (Vec2){(double)mx, (double)my};
        }
    }
    else if (e->type == SDL_KEYDOWN){
        SDL_Keycode k = e->key.keysym.sym;
        if (k == SDLK_ESCAPE || k == SDLK_q) running = 0;
        else if (k == SDLK_p) paused = !paused;
        else if (k == SDLK_f) show_trails = !show_trails;
        else if (k == SDLK_r) clear_universe();
        else if (k == SDLK_s) { if(!save_universe("universe.save")) perror("save"); }
        else if (k == SDLK_l) { if(!load_universe("universe.save")) perror("load"); }
        else if (k == SDLK_h) { help_overlay = !help_overlay; }
        else if (k == SDLK_m) { size_by_mass = !size_by_mass; }
        else if (k == SDLK_n) { spawn_spiral_galaxy((Vec2){0,0}, 3 + (rng_u32()%3), 1800, 20000.0); }
        else if (k == SDLK_g) { G_CONST = fmax(1.0, G_CONST*0.9); printf("G=%g\n", G_CONST);} // dec gravity
        else if (k == SDLK_SEMICOLON) { G_CONST = fmin(500.0, G_CONST*1.1); printf("G=%g\n", G_CONST);} // inc gravity
        else if (k == SDLK_t) { TIME_SCALE = fmax(0.05, TIME_SCALE*0.8); printf("time=%g\n", TIME_SCALE);} // slower
        else if (k == SDLK_y) { TIME_SCALE = fmin(5.0, TIME_SCALE*1.25); printf("time=%g\n", TIME_SCALE);} // faster
    }
}

// ------------------- Main -------------------

int main(int argc, char **argv){
    (void)argc; (void)argv;
    srand((unsigned)time(NULL)); rng_u32(); rng_u32();

    if (SDL_Init(SDL_INIT_VIDEO|SDL_INIT_TIMER) != 0){
        fprintf(stderr, "SDL_Init failed: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window *W = SDL_CreateWindow("Player's Universe Simulator", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, WIN_W, WIN_H, SDL_WINDOW_SHOWN);
    if (!W){ fprintf(stderr, "CreateWindow failed: %s\n", SDL_GetError()); return 1; }

    SDL_Renderer *R = SDL_CreateRenderer(W, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!R){ fprintf(stderr, "CreateRenderer failed: %s\n", SDL_GetError()); return 1; }
    SDL_SetRenderDrawBlendMode(R, SDL_BLENDMODE_BLEND);

    print_help_once();

    // Initial scene: tiny system
    add_body((Vec2){0,0}, (Vec2){0,0}, 8000.0, 8.0, (SDL_Color){255,230,180,255}, 0);
    for(int i=0;i<40;i++){
        double ang = frand()*2*M_PI; double r = 60+frand()*220; Vec2 p={r*cos(ang), r*sin(ang)};
        double v = sqrt(G_CONST*8000.0/r);
        Vec2 tang={-sin(ang), cos(ang)};
        Vec2 vel=scale(tang, v*(0.9+0.2*frand()));
        SDL_Color c={(Uint8)(120+120*frand()),(Uint8)(140+115*frand()),(Uint8)(200),255};
        add_body(p, vel, 1.0+2.0*frand(), 2.0+1.5*frand(), c, 0);
    }

    Uint64 prev = SDL_GetPerformanceCounter();
    double acc = 0.0; const double fixed_dt = 0.016; // 60 Hz

    while(running){
        SDL_Event e; while(SDL_PollEvent(&e)) handle_event(&e, W);
        Uint64 now = SDL_GetPerformanceCounter();
        double dt = (double)(now - prev) / SDL_GetPerformanceFrequency();
        prev = now;

        if (!paused){
            acc += dt * TIME_SCALE;
            while (acc >= fixed_dt){ integrate(fixed_dt); acc -= fixed_dt; }
        }
        render(R);
    }

    SDL_DestroyRenderer(R); SDL_DestroyWindow(W); SDL_Quit();
    return 0;
}

