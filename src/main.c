#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>
#include <zephyr/sys/printk.h>

#define PERIODO_MS 50   /* 20 Hz, igual ao gerar_graficos.py */

/* Uma amostra de aceleração (m/s²) com o instante em que foi lida */
struct amostra_acc {
    int64_t t_ms;
    float a[3];         /* x, y, z */
};

static const struct device *const accel =
    DEVICE_DT_GET(DT_NODELABEL(mma8451q));

/* Fila: 8 posições de tamanho sizeof(struct amostra_acc), alinhada a 4 bytes */
K_MSGQ_DEFINE(fila_acc, sizeof(struct amostra_acc), 8, 4);

static void thread_aquisicao(void *p1, void *p2, void *p3)
{
    ARG_UNUSED(p1); ARG_UNUSED(p2); ARG_UNUSED(p3);

    if (!device_is_ready(accel)) {
        printk("ERRO: acelerometro nao esta pronto!\n");
        return;
    }

    int64_t proximo = k_uptime_get();

    while (1) {
        /* Dorme até um instante ABSOLUTO: o período não acumula atraso */
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

        /* K_NO_WAIT: a aquisição NUNCA pode ficar bloqueada esperando */
        if (k_msgq_put(&fila_acc, &s, K_NO_WAIT) != 0) {
            printk("# fila cheia, amostra descartada\n");
        }
    }
}

/* Cria e inicia a thread automaticamente no boot.
   Args: nome, pilha(bytes), função, p1, p2, p3, prioridade, opções, atraso */
K_THREAD_DEFINE(tid_aquisicao, 1024, thread_aquisicao,
                NULL, NULL, NULL, 5, 0, 0);

int main(void)
{
    struct amostra_acc s;

    /* TEMPORÁRIO: consumidor de teste */
    while (1) {
        k_msgq_get(&fila_acc, &s, K_FOREVER);
        printk("%lld, %.4f, %.4f, %.4f\n",
               s.t_ms, (double)s.a[0], (double)s.a[1], (double)s.a[2]);
    }
    return 0;
}