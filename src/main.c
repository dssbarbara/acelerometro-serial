#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

#define PERIODO_MS 50   /* 20 Hz, igual ao gerar_graficos.py */
#define N_CALIB    40   /* 40 amostras x 50 ms = 2 s de calibração */

/* Aceleração bruta, saída da thread de aquisição */
struct amostra_acc {
    int64_t t_ms;
    float a[3];                 /* x, y, z em m/s² */
};

/* Resultado cinemático, saída da thread de integração */
struct amostra_cin {
    float a[3];                 /* aceleração já sem offset */
    float v[3];                 /* velocidade (m/s) */
    float p[3];                 /* posição (m) */
};

static const struct device *const accel =
    DEVICE_DT_GET(DT_NODELABEL(mma8451q));

K_MSGQ_DEFINE(fila_acc, sizeof(struct amostra_acc), 8, 4);  /* aquisição -> integração */
K_MSGQ_DEFINE(fila_cin, sizeof(struct amostra_cin), 8, 4);  /* integração -> serial */

/* ============ THREAD 1: AQUISIÇÃO (prioridade 5) ============ */
static void thread_aquisicao(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    if (!device_is_ready(accel)) {
        printk("# ERRO: acelerometro nao esta pronto!\n");
        return;
    }

    int64_t proximo = k_uptime_get();

    while (1) {
        proximo += PERIODO_MS;
        k_sleep(K_TIMEOUT_ABS_MS(proximo));

        if (sensor_sample_fetch(accel) != 0) {
            continue;
        }

        struct sensor_value x, y, z;
        sensor_channel_get(accel, SENSOR_CHAN_ACCEL_X, &x);
        sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Y, &y);
        sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Z, &z);

        struct amostra_acc s = {
            .t_ms = k_uptime_get(),
            .a = { sensor_value_to_float(&x),
                   sensor_value_to_float(&y),
                   sensor_value_to_float(&z) },
        };

        if (k_msgq_put(&fila_acc, &s, K_NO_WAIT) != 0) {
            printk("# fila_acc cheia, amostra descartada\n");
        }
    }
}

/* ============ THREAD 2: INTEGRAÇÃO (prioridade 6) ============ */
static void thread_integracao(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    struct amostra_acc s;
    float offset[3] = {0, 0, 0};

    /* --- Calibração: média com a placa parada --- */
    printk("# Calibrando %d amostras, mantenha a placa parada...\n", N_CALIB);
    for (int i = 0; i < N_CALIB; i++) {
        k_msgq_get(&fila_acc, &s, K_FOREVER);
        for (int k = 0; k < 3; k++) {
            offset[k] += s.a[k];
        }
    }
    for (int k = 0; k < 3; k++) {
        offset[k] /= N_CALIB;
    }
    printk("# Calibrado! Pode mover a placa.\n");

    /* --- Estado da integração --- */
    float a_ant[3] = {0, 0, 0};
    float v[3]     = {0, 0, 0};
    float p[3]     = {0, 0, 0};
    int64_t t_ant  = s.t_ms;    /* último instante da calibração */

    while (1) {
        k_msgq_get(&fila_acc, &s, K_FOREVER);

        float dt = (float)(s.t_ms - t_ant) / 1000.0f;   /* ms -> s */
        t_ant = s.t_ms;

        struct amostra_cin out;

        for (int k = 0; k < 3; k++) {
            float a     = s.a[k] - offset[k];
            float v_ant = v[k];

            /* Regra dos trapézios, duas vezes */
            v[k] += 0.5f * (a_ant[k] + a) * dt;
            p[k] += 0.5f * (v_ant + v[k]) * dt;

            a_ant[k] = a;
            out.a[k] = a;
            out.v[k] = v[k];
            out.p[k] = p[k];
        }

        if (k_msgq_put(&fila_cin, &out, K_NO_WAIT) != 0) {
            printk("# fila_cin cheia, amostra descartada\n");
        }
    }
}

/* ============ THREAD 3: ENVIO SERIAL (prioridade 7) ============ */
static void thread_serial(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    struct amostra_cin c;
    bool cabecalho_enviado = false;

    while (1) {
        k_msgq_get(&fila_cin, &c, K_FOREVER);

        if (!cabecalho_enviado) {
            printk("ax,ay,az,vx,vy,vz,px,py,pz\n");
            cabecalho_enviado = true;
        }

        printk("%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f\n",
               (double)c.a[0], (double)c.a[1], (double)c.a[2],
               (double)c.v[0], (double)c.v[1], (double)c.v[2],
               (double)c.p[0], (double)c.p[1], (double)c.p[2]);
    }
}

/* Nome, pilha, função, p1, p2, p3, prioridade, opções, atraso */
K_THREAD_DEFINE(tid_aquisicao,  1024, thread_aquisicao,  NULL, NULL, NULL, 5, 0, 0);
K_THREAD_DEFINE(tid_integracao, 1024, thread_integracao, NULL, NULL, NULL, 6, 0, 0);
K_THREAD_DEFINE(tid_serial,     2048, thread_serial,     NULL, NULL, NULL, 7, 0, 0);

int main(void)
{
    /* Tudo acontece nas threads; o main não tem mais trabalho */
    return 0;
}