/* A minimal test harness: CHECK records failures and keeps going. */
#ifndef LBTEST_H
#define LBTEST_H

#include <math.h>
#include <stdio.h>

static int lbt_failures = 0, lbt_checks = 0;

#define CHECK(cond, ...)                                                          \
    do {                                                                          \
        lbt_checks++;                                                             \
        if (!(cond)) {                                                            \
            lbt_failures++;                                                       \
            printf("  FAIL %s:%d: %s: ", __FILE__, __LINE__, #cond);              \
            printf(__VA_ARGS__);                                                  \
            printf("\n");                                                         \
        }                                                                         \
    } while (0)

#define CHECK_NEAR(a, b, tol)                                                     \
    CHECK(fabs((double)(a) - (double)(b)) <= (tol), "%g vs %g (tol %g)", (double)(a), (double)(b), (double)(tol))

#define RUN(fn)                                                                   \
    do {                                                                          \
        int before = lbt_failures;                                                \
        fn();                                                                     \
        printf("%s %s\n", lbt_failures == before ? "ok  " : "FAIL", #fn);         \
    } while (0)

#define DONE()                                                                    \
    do {                                                                          \
        printf("%d checks, %d failures\n", lbt_checks, lbt_failures);             \
        return lbt_failures ? 1 : 0;                                              \
    } while (0)

#endif
