import { promises as fs } from 'fs';

function x_dot(t, x, w_) {
    const w_max = 15000.0;
    const m = 0.25;
    const g = 9.81;
    const l = 0.1;
    const kf = 1.744e-08;
    const Iz = 2e-4;
    const tal = 0.005;
    const P = [[0], [-m * g]];

    const w = x.slice(0, 2);
    const r = x.slice(2, 4);
    const v = x.slice(4, 6);
    const phi = x[6];
    const ome = x[7];

    const f1 = kf * Math.pow(w[0], 2);
    const f2 = kf * Math.pow(w[1], 2);

    const Tc = l * (f1 - f2);

    const Fc_B = [[0], [(f1 + f2)]];
    const D_RB = [
        [Math.cos(phi), -Math.sin(phi)],
        [Math.sin(phi), Math.cos(phi)]
    ];

    const w_dot = w.map((val, idx) => (-val + w_[idx]) / tal);
    const r_dot = v;
    const D_RB_Fc_B = [
        D_RB[0][0] * Fc_B[0][0] + D_RB[0][1] * Fc_B[1][0],
        D_RB[1][0] * Fc_B[0][0] + D_RB[1][1] * Fc_B[1][0]
    ];
    const v_dot = [
        (1 / m) * D_RB_Fc_B[0] + P[0][0],
        (1 / m) * D_RB_Fc_B[1] + P[1][0]
    ];
    const phi_dot = [ome];
    const ome_dot = [Tc / Iz];

    const xkp1 = w_dot.concat(r_dot, v_dot, phi_dot, ome_dot);
    return xkp1;
}

function rk4(tk, h, xk, uk) {
    const k1 = x_dot(tk, xk, uk);
    const k2 = x_dot(tk + h / 2.0, xk.map((val, idx) => val + h * k1[idx] / 2.0), uk);
    const k3 = x_dot(tk + h / 2.0, xk.map((val, idx) => val + h * k2[idx] / 2.0), uk);
    const k4 = x_dot(tk + h, xk.map((val, idx) => val + h * k3[idx]), uk);

    const xkp1 = xk.map((val, idx) => val + (h / 6.0) * (k1[idx] + 2 * k2[idx] + 2 * k3[idx] + k4[idx]));

    return xkp1;
}

const h = 1e-3;
const Ts = 10e-3;
const fTh = Ts / h;
const maxT = 60;
const tc = Array.from({ length: maxT / h }, (_, i) => i * h);
const td = Array.from({ length: maxT / Ts }, (_, i) => i * Ts);
const tam = tc.length;
let j = 0;

let x = Array.from({ length: 8 }, () => Array(tam).fill(0));
x.forEach((_, idx) => {
    x[idx][0] = idx < 6 ? 0 : 0 * Math.PI / 180;
});

const w_ = Array.from({ length: 2 }, () => Array(td.length).fill(0));
const Phi_ = Array(td.length).fill(0);
const WP_ = Array.from({ length: 2 }, () => Array(td.length).fill(0));

const eP_ = Array.from({ length: 2 }, () => Array(td.length).fill(0));
const eV_ = Array.from({ length: 2 }, () => Array(td.length).fill(0));
const ePhi_ = Array(td.length).fill(0);
const eOme_ = Array(td.length).fill(0);

let ePm1 = 0;
let eVm1 = 0;

const m = 0.25;
const g = 9.81;
const l = 0.1;
const kf = 1.744e-08;
const Iz = 2e-4;
const tal = 0.05;
const Fe = [-m * g];

const phi_max = 15 * Math.PI / 180;
const w_max = 15000;
const Fc_max = kf * Math.pow(w_max, 2);
const Tc_max = l * kf * Math.pow(w_max, 2);

const r_ = [
    [0, 15, -50, -20, 10],
    [10, 10, 2, 15, 0]
];
const r_ID = 0;
const r_IDN = 4;

for (let k = 0; k < tam - 1; k++) {
    if (k % fTh === 0) {
        const r_k = x.slice(2, 4)[0];
        const v_k = x.slice(4, 6)[0];
        const phi_k = x[6][0];
        const ome_k = x[7][0];

        const v_ = [0, 0];

        const kpP = [.075];
        const kdP = [0.25];
        const eP = r_.map((val, idx) => val[r_ID] - r_k[idx]);
        const eV = v_.map((val, idx) => v_[idx] - v_k[idx]);
        WP_.forEach((row, idx) => row[j] = r_[idx][r_ID]);
        eP_.forEach((row, idx) => row[j] = eP[idx]);
        eV_.forEach((row, idx) => row[j] = eV[idx]);

        if (Math.sqrt(eP.reduce((acc, val) => acc + Math.pow(val, 2), 0)) < 0.1 && r_ID < r_IDN) {
            r_ID += 1;
        }

        const Fx = kpP * eP[0] + kdP * eV[0];
        const Fy = kpP * eP[1] + kdP * eV[1] - Fe;
        const clampedFy = Math.min(Math.max(0.2 * Fc_max, Fy), 0.8 * Fc_max);

        let phi_ = Math.atan2(-Fx, clampedFy);

        if (Math.abs(phi_) > phi_max) {
            const signal = phi_ / Math.abs(phi_);
            phi_ = signal * phi_max;

            Fx = clampedFy * Math.tan(phi_);
        }
        Phi_[j] = phi_;

        const Fc = Math.sqrt(Math.pow(Fx, 2) + Math.pow(clampedFy, 2));
        const f12 = [Fc / 2.0, Fc / 2.0];

        const kpA = [.75];
        const kdA = [0.05];
        const ePhi = phi_ - phi_k;
        const eOme = 0 - ome_k;

        ePhi_[j] = ePhi;
        eOme_[j] = eOme;

        let Tc = kpA * ePhi + kdA * eOme;
        Tc = Math.min(Math.max(-0.4 * Tc_max, Tc), 0.4 * Tc_max);

        const df12 = Math.abs(Tc) / 2.0;

        if (Tc >= 0.0) {
            f12[0] += df12;
            f12[1] -= df12;
        } else {
            f12[0] -= df12;
            f12[1] += df12;
        }

        const w1_ = Math.sqrt(f12[0] / kf);
        const w2_ = Math.sqrt(f12[1] / kf);

        const w1 = Math.min(Math.max(0, w1_), w_max);
        const w2 = Math.min(Math.max(0, w2_), w_max);

        w_[0][j] = w1;
        w_[1][j] = w2;

        j++;
    }

    x = rk4(tc[k], h, x.map(row => row[k]), w_.map(row => row[j - 1]));
}

const f = Array.from({ length: 3 }, () => Array(tam).fill(0));

for (let k = 0; k < tam; k++) {
    const w = x.slice(0, 2)[0];
    f[0][k] = kf * Math.pow(w[0], 2);
    f[1][k] = kf * Math.pow(w[1], 2);
    f[2][k] = f[0][k] + f[1][k];
}

const VecXf = tc.map((val, idx) => [val, ...x.map(row => row[idx]), ...f.map(row => row[idx])]);
const VecControl = td.map((val, idx) => [
    val,
    ...WP_.map(row => row[idx]),
    ...w_.map(row => row[idx]),
    ...eP_.map(row => row[idx]),
    ...eV_.map(row => row[idx]),
    ePhi_[idx],
    eOme_[idx]
]);
async function salvarDadosCSV(dados, nomeArquivo) {
    const csvContent = dados.map(row => row.join(',')).join('\n');
    try {
        await fs.writeFile(nomeArquivo, csvContent);
        console.log(`Dados salvos em ${nomeArquivo}`);
    } catch (error) {
        console.error(`Erro ao salvar os dados: ${error}`);
    }
}

salvarDadosCSV(VecXf, 'dados_simulacao.csv');
salvarDadosCSV(VecControl, 'dados_controle.csv');