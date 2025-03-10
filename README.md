# Optimal-Flight-Control

## Descrizione
**Optimal-Flight-Control** è un progetto che implementa tecniche di controllo ottimo (**LQR**) e controllo predittivo (**MPC**) per la gestione dell'atterraggio di un aereo sperimentale. Il sistema è progettato per garantire stabilità e precisione in presenza di disturbi atmosferici, utilizzando un modello dinamico discreto per la simulazione e l'ottimizzazione della traiettoria.

## Struttura del progetto
- **`setup_mpc.m`** → Configura il controllore MPC e ne salva i parametri.
- **`lqr_trajectory.m`** → Calcola la traiettoria ottima utilizzando il controllo LQR.
- **`mpc_control.m`** → Esegue il controllo MPC in loop chiuso con disturbi atmosferici.
- **`data/`** → Contiene i file salvati delle simulazioni (parametri, traiettorie, risultati, etc.).
- **`functions/`** → Funzioni di supporto per la dinamica e il controllo.

## Requisiti
- MATLAB con Control System Toolbox.
- Simulink (opzionale, per eventuali test avanzati).

## Come usare il progetto
1. **Configurare il controllore MPC**:
   ```matlab
   setup_mpc
   ```
2. **Calcolare la traiettoria ottima con LQR**:
   ```matlab
   lqr_trajectory
   ```
3. **Eseguire il controllo MPC per l'atterraggio**:
   ```matlab
   mpc_control
   ```


