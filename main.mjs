import { fileURLToPath } from 'url';
import { dirname } from 'path';

const __filename = fileURLToPath(import.meta.url);
const __dirname = dirname(__filename);
import { writeFile } from 'fs/promises';

class DroneSimulation {
    initializeVectors() {
        const tam = this.tam; // Tamanho dos arrays de tempo

        this.x = Array.from({ length: 8 }, () => new Array(tam).fill(0)); // Matriz de estado
        this.x[0][0] = 0;
        this.x[1][0] = 0;
        this.x[2][0] = 0;
        this.x[3][0] = 0;
        this.x[4][0] = 0;
        this.x[5][0] = 0;
        this.x[6][0] = 0 * Math.PI / 180;
        this.x[7][0] = 0 * Math.PI / 180;

        // Inicialização dos vetores de controle e erros
        this.w_ = Array.from({ length: 2 }, () => new Array(this.td.length).fill(0));
        this.Phi_ = new Array(this.td.length).fill(0);
        this.WP_ = Array.from({ length: 2 }, () => new Array(this.td.length).fill(0));
        this.eP_ = Array.from({ length: 2 }, () => new Array(this.td.length).fill(0));
        this.eV_ = Array.from({ length: 2 }, () => new Array(this.td.length).fill(0));
        this.ePhi_ = new Array(this.td.length).fill(0);
        this.eOme_ = new Array(this.td.length).fill(0);

        this.ePm1 = 0; // Erro posição k-1 (passo anterior)
        this.eVm1 = 0; // Erro atitude k-1 (passo anterior)
    }

    constructor() {
        // Definição dos parâmetros e constantes
        this.h = 1e-3; // passo da simulação de tempo contínuo
        this.Ts = 10e-3; // intervalo de atuação do controlador
        this.fTh = this.Ts / this.h;
        this.maxT = 60;
        this.tam = Math.floor(this.maxT / this.h);
        this.j = 0;

        // Vetor de estados
        this.x = Array.from({ length: 8 }, () => new Array(this.tam).fill(0));
        this.x[0][0] = 0;
        this.x[1][0] = 0;
        this.x[2][0] = 0;
        this.x[3][0] = 0;
        this.x[4][0] = 0;
        this.x[5][0] = 0;
        this.x[6][0] = 0 * Math.PI / 180;
        this.x[7][0] = 0 * Math.PI / 180;

        // Parâmetros do sistema de controle
        this.w_ = Array.from({ length: 2 }, () => new Array(this.tam).fill(0)); // Vetor de controle relativo à rotação
        this.Phi_ = new Array(this.tam).fill(0); // Vetor de comando de Atitude
        this.WP_ = Array.from({ length: 2 }, () => new Array(this.tam).fill(0)); // Vetor de comando de Waypoint
        this.eP_ = Array.from({ length: 2 }, () => new Array(this.tam).fill(0)); // Vetor dos erros de posição
        this.eV_ = Array.from({ length: 2 }, () => new Array(this.tam).fill(0)); // Vetor dos erros de velocidade
        this.ePhi_ = new Array(this.tam).fill(0); // Vetor dos erros de atitude
        this.eOme_ = new Array(this.tam).fill(0); // Vetor dos erros de velocidade angular

        this.ePm1 = 0; // erro posição k-1 (passo anterior)
        this.eVm1 = 0; // erro atitude k-1 (passo anterior)

        // Constantes do modelo
        this.m = 0.25; // massa
        this.g = 9.81; // aceleração da gravidade
        this.l = 0.1; // tamanho
        this.kf = 1.744e-08; // constante de força
        this.Iz = 2e-4; // momento de inércia
        this.tal = 0.05;
        this.Fe = -this.m * this.g;

        // Restrições do controle
        this.phi_max = 15 * Math.PI / 180; // ângulo máximo
        this.w_max = 15000;
        this.Fc_max = this.kf * Math.pow(this.w_max, 2); // Força de controle máximo
        this.Tc_max = this.l * this.kf * Math.pow(this.w_max, 2);

        // Waypoints
        this.r_ = [
            [0, 15, -50, -20, 10],
            [10, 10, 2, 15, 0]
        ];
        this.r_ID = 0;
        this.r_IDN = 4;
    }

    xDot(t, x, w_) {
        // Parâmetros
        console.log('x:', x);
        console.log('w:', w_);
        const w_max = 15000; // velocidade máxima do motor
        const m = 0.25; // massa
        const g = 9.81; // aceleração da gravidade
        const l = 0.1; // tamanho
        const kf = 1.744e-08; // constante de força
        const Iz = 2e-4; // momento de inércia
        const tal = 0.005;

        const P = [0, -m * g];

        // Estados atuais
        const w = x.slice(0, 2);
        const r = x.slice(2, 4);
        const v = x.slice(4, 6);
        const phi = x[6];
        const ome = x[7];

        // Forças
        const f1 = kf * Math.pow(w[0], 2);
        const f2 = kf * Math.pow(w[1], 2);

        // Torque
        const Tc = l * (f1 - f2);

        // Força de controle
        const Fc_B = [0, f1 + f2];

        // Matriz de atitude
        const D_RB = [
            [Math.cos(phi), -Math.sin(phi)],
            [Math.sin(phi), Math.cos(phi)]
        ];

        // Derivadas
        const w_dot = w.map((w_i, i) => (-w_i + w_[i]) / tal);
        const r_dot = v;
        const v_dot = [
            (1 / m) * (D_RB[0][0] * Fc_B[0] + D_RB[0][1] * Fc_B[1] + P[0]),
            (1 / m) * (D_RB[1][0] * Fc_B[0] + D_RB[1][1] * Fc_B[1] + P[1])
        ];
        const phi_dot = [ome];
        const ome_dot = [Tc / Iz];

        const xkp1 = [...w_dot, ...r_dot, ...v_dot, ...phi_dot, ...ome_dot];
        return xkp1;
    }

    rk4(tk, h, xk, uk) {
        const k1 = this.xDot(tk, xk, uk);
        const k2 = this.xDot(tk + h / 2.0, xk.map((x_i, i) => x_i + h * k1[i] / 2.0), uk);
        const k3 = this.xDot(tk + h / 2.0, xk.map((x_i, i) => x_i + h * k2[i] / 2.0), uk);
        const k4 = this.xDot(tk + h, xk.map((x_i, i) => x_i + h * k3[i]), uk);

        const xkp1 = xk.map((x_i, i) => x_i + (h / 6.0) * (k1[i] + 2 * k2[i] + 2 * k3[i] + k4[i]));
        return xkp1;
    }

    async generateAndSaveResults(tc, positionsX, positionsY, errorsP, rotorSpeed, displacement, velocity, attitude) {
        if (!positionsX || !positionsY || positionsX.length === 0 || positionsY.length === 0) {
            console.error('Dados de posição ausentes ou inválidos.');
            return;
        }

        const results = [
            ['Tempo', 'Posição X', 'Posição Y']
        ];

        for (let i = 0; i < tc.length; i++) {
            results.push([
                tc[i],
                positionsX[i],
                positionsY[i]
            ]);
        }

        const csvContent = results.map(row => row.join(',')).join('\n');

        try {
            await writeFile(__dirname + '/simulation_results_2d.csv', csvContent);
            console.log('Resultados da simulação 2D salvos.');
        } catch (err) {
            console.error('Erro ao salvar os resultados da simulação 2D:', err);
        }
    }

    simulate() {
        for (let k = 0; k < this.tam - 1; k++) {
            // Sistema de controle
            if (k % this.fTh === 0) {
                // Extrai os dados do vetor
                let r_k = [this.x[2][k], this.x[3][k]]; // Posição atual
                let v_k = [this.x[4][k], this.x[5][k]]; // Velocidade atual
                let phi_k = this.x[6][k]; // Ângulo de inclinação atual
                let ome_k = this.x[7][k]; // Velocidade angular atual

                // Lógica de controle de posição
                let kpP = 0.075;
                let kdP = 0.25;
                let eP = [this.r_[0][this.r_ID] - r_k[0], this.r_[1][this.r_ID] - r_k[1]];
                let eV = [0 - v_k[0], 0 - v_k[1]];
                this.WP_[0][this.j] = this.r_[0][this.r_ID];
                this.WP_[1][this.j] = this.r_[1][this.r_ID];
                this.eP_[0][this.j] = eP[0];
                this.eP_[1][this.j] = eP[1];
                this.eV_[0][this.j] = eV[0];
                this.eV_[1][this.j] = eV[1];

                // Lógica de controle de Waypoint
                if (Math.sqrt(eP[0] * eP[0] + eP[1] * eP[1]) < 0.1 && this.r_ID < this.r_IDN) {
                    this.r_ID++;
                    console.log("Buscar Waypoint: ", this.r_ID);
                }

                let Fx = kpP * eP[0] + kdP * eV[0];
                let Fy = kpP * eP[1] + kdP * eV[1] - this.Fe;
                Fy = Math.max(0.2 * this.Fc_max, Math.min(Fy, 0.8 * this.Fc_max));

                // Lógica de controle de atitude
                let phi_ = Math.atan2(-Fx, Fy);
                if (Math.abs(phi_) > this.phi_max) {
                    let signal = phi_ / Math.abs(phi_);
                    phi_ = signal * this.phi_max;
                    Fx = Fy * Math.tan(phi_);
                }
                this.Phi_[this.j] = phi_;

                let Fxy = [Fx, Fy];
                let Fc = Math.sqrt(Fxy[0] * Fxy[0] + Fxy[1] * Fxy[1]);
                let f12 = [Fc / 2.0, Fc / 2.0];

                // Lógica de controle de ângulo e velocidade angular
                let kpA = 0.75;
                let kdA = 0.05;
                let ePhi = phi_ - phi_k;
                let eOme = 0 - ome_k;
                this.ePhi_[this.j] = ePhi;
                this.eOme_[this.j] = eOme;

                let Tc = kpA * ePhi + kdA * eOme;
                Tc = Math.max(-0.4 * this.Tc_max, Math.min(Tc, 0.4 * this.Tc_max));

                let df12 = Math.abs(Tc) / 2.0;
                if (Tc >= 0.0) {
                    f12[0] = f12[0] + df12;
                    f12[1] = f12[1] - df12;
                } else {
                    f12[0] = f12[0] - df12;
                    f12[1] = f12[1] + df12;
                }

                let w1_ = Math.sqrt(f12[0] / this.kf);
                let w2_ = Math.sqrt(f12[1] / this.kf);

                let w1 = Math.max(0, Math.min(w1_, this.w_max));
                let w2 = Math.max(0, Math.min(w2_, this.w_max));

                this.w_[0][this.j] = w1;
                this.w_[1][this.j] = w2;

                this.j++;
            }

            // Simulação um passo à frente
            let tk = k * this.h;
            this.x[0][k + 1] = this.x[0][k];
            this.x[1][k + 1] = this.x[1][k];
            this.x[2][k + 1] = this.x[2][k];
            this.x[3][k + 1] = this.x[3][k];
            this.x[4][k + 1] = this.x[4][k];
            this.x[5][k + 1] = this.x[5][k];
            this.x[6][k + 1] = this.x[6][k];
            this.x[7][k + 1] = this.x[7][k];

            let xk = [
                this.x[0][k],
                this.x[1][k],
                this.x[2][k],
                this.x[3][k],
                this.x[4][k],
                this.x[5][k],
                this.x[6][k],
                this.x[7][k]
            ];
            let uk = [this.w_[0][this.j - 1], this.w_[1][this.j - 1]];

            this.x[0][k + 1] = this.x[0][k];
            this.x[1][k + 1] = this.x[1][k];
            this.x[2][k + 1] = this.x[2][k];
            this.x[3][k + 1] = this.x[3][k];
            this.x[4][k + 1] = this.x[4][k];
            this.x[5][k + 1] = this.x[5][k];
            this.x[6][k + 1] = this.x[6][k];
            this.x[7][k + 1] = this.x[7][k];

            let k1 = this.xDot(tk, xk, uk);
            let k2 = this.xDot(tk + this.h / 2.0, xk + this.h * k1 / 2.0, uk);
            let k3 = this.xDot(tk + this.h / 2.0, xk + this.h * k2 / 2.0, uk);
            let k4 = this.xDot(tk + this.h, xk + this.h * k3, uk);

            this.x[0][k + 1] = this.x[0][k] + (this.h / 6.0) * (k1[0] + 2 * k2[0] + 2 * k3[0] + k4[0]);
            this.x[1][k + 1] = this.x[1][k] + (this.h / 6.0) * (k1[1] + 2 * k2[1] + 2 * k3[1] + k4[1]);
            this.x[2][k + 1] = this.x[2][k] + (this.h / 6.0) * (k1[2] + 2 * k2[2] + 2 * k3[2] + k4[2]);
            this.x[3][k + 1] = this.x[3][k] + (this.h / 6.0) * (k1[3] + 2 * k2[3] + 2 * k3[3] + k4[3]);
            this.x[4][k + 1] = this.x[4][k] + (this.h / 6.0) * (k1[4] + 2 * k2[4] + 2 * k3[4] + k4[4]);
            this.x[5][k + 1] = this.x[5][k] + (this.h / 6.0) * (k1[5] + 2 * k2[5] + 2 * k3[5] + k4[5]);
            this.x[6][k + 1] = this.x[6][k] + (this.h / 6.0) * (k1[6] + 2 * k2[6] + 2 * k3[6] + k4[6]);
            this.x[7][k + 1] = this.x[7][k] + (this.h / 6.0) * (k1[7] + 2 * k2[7] + 2 * k3[7] + k4[7]);
        }

        // Processamento de variáveis intermediárias
        // ...

        // Visualização dos resultados
        // ...
    }

}
// Cria uma instância da classe DroneSimulation
const simulation = new DroneSimulation();

// Chama o método simulate para iniciar a simulação
simulation.simulate();