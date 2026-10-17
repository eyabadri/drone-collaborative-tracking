#!/usr/bin/env python3
"""
PFA — Contrôleur PID générique avec anti-windup et clamping.
Fichier : phase_suivi/scripts/pid_controller.py

Chaque axe instancie un PIDController indépendant.
La méthode compute() retourne le signal de commande ET les 3 termes
P/I/D séparément pour faciliter le logging et le tuning.
"""

import time
import math


class PIDController:
    """
    Contrôleur PID discret avec :
      - Anti-windup par clamping de l'intégrale
      - Filtrage du terme dérivé (filtre passe-bas 1er ordre)
      - Saturation de la sortie
      - Reset propre lors des transitions d'état

    Args:
        kp   (float): Gain proportionnel
        ki   (float): Gain intégral  (0 = pas d'intégrale → PD)
        kd   (float): Gain dérivé    (0 = pas de dérivée  → PI)
        integral_limit (float|None): Saturation de l'intégrale (anti-windup)
        output_limit   (float|None): Saturation de la sortie finale
        derivative_filter (float)  : Constante de filtre dérivé [0..1]
                                     0 = pas de filtre, 0.7 = lissage fort
    """

    def __init__(
        self,
        kp: float,
        ki: float = 0.0,
        kd: float = 0.0,
        integral_limit: float | None = None,
        output_limit: float | None = None,
        derivative_filter: float = 0.1,
    ):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral_limit = integral_limit
        self.output_limit = output_limit
        self.derivative_filter = derivative_filter

        # État interne
        self._integral: float = 0.0
        self._prev_error: float = 0.0
        self._prev_derivative: float = 0.0
        self._last_time: float = 0.0
        self._initialized: bool = False

    # ------------------------------------------------------------------
    def compute(self, error: float, dt: float | None = None):
        """
        Calcule la commande PID pour une erreur donnée.

        Args:
            error (float): Erreur = setpoint - mesure
            dt    (float): Pas de temps (s). Si None, utilise l'horloge réelle.

        Returns:
            tuple: (output, p_term, i_term, d_term)
                   output  = commande totale (saturée si output_limit défini)
                   p_term  = contribution proportionnelle
                   i_term  = contribution intégrale
                   d_term  = contribution dérivée
        """
        now = time.time()

        if dt is None:
            if not self._initialized:
                dt = 0.05  # valeur par défaut 20 Hz
            else:
                dt = max(1e-4, min(now - self._last_time, 1.0))

        self._last_time = now
        self._initialized = True

        # ── Terme P ───────────────────────────────────────────────────
        p_term = self.kp * error

        # ── Terme I avec anti-windup ──────────────────────────────────
        self._integral += error * dt
        if self.integral_limit is not None:
            self._integral = max(
                -self.integral_limit,
                min(self._integral, self.integral_limit)
            )
        i_term = self.ki * self._integral

        # ── Terme D avec filtre passe-bas ─────────────────────────────
        raw_derivative = (error - self._prev_error) / dt if dt > 1e-4 else 0.0
        alpha = self.derivative_filter
        filtered_d = (1.0 - alpha) * raw_derivative + alpha * self._prev_derivative
        d_term = self.kd * filtered_d

        self._prev_error = error
        self._prev_derivative = filtered_d

        # ── Sortie totale + saturation ────────────────────────────────
        output = p_term + i_term + d_term
        if self.output_limit is not None:
            output = max(-self.output_limit, min(output, self.output_limit))

        return output, p_term, i_term, d_term

    # ------------------------------------------------------------------
    def reset(self):
        """
        Remet à zéro l'intégrale et l'état dérivé.
        À appeler lors des transitions d'état (hover → tracking, etc.)
        """
        self._integral = 0.0
        self._prev_error = 0.0
        self._prev_derivative = 0.0
        self._initialized = False

    # ------------------------------------------------------------------
    def set_gains(self, kp: float, ki: float, kd: float):
        """Modifie les gains à chaud (pour le tuning en ligne)."""
        self.kp = kp
        self.ki = ki
        self.kd = kd

    # ------------------------------------------------------------------
    @property
    def integral(self) -> float:
        """Valeur courante de l'intégrale (utile pour le monitoring)."""
        return self._integral

    # ------------------------------------------------------------------
    def __repr__(self) -> str:
        return (
            f"PIDController(kp={self.kp}, ki={self.ki}, kd={self.kd}, "
            f"int_lim={self.integral_limit}, out_lim={self.output_limit})"
        )
