#!/usr/bin/env python3
"""
Hand-Eye Calibration Script

Estimates the rigid transformation from camera frame to robot base frame
using recorded calibration pairs. Performs hold-out cross-validation to
evaluate the calibration quality.

Usage:
    python calibrate_hand_eye.py calibration_pairs_YYYYMMDD_HHMMSS.yaml
    python calibrate_hand_eye.py calibration_pairs_YYYYMMDD_HHMMSS.yaml --test-ratio 0.3
    python calibrate_hand_eye.py calibration_pairs_YYYYMMDD_HHMMSS.yaml --k-fold 5
"""

import argparse
import yaml
import numpy as np
from pathlib import Path


def load_calibration_pairs(yaml_path):
    """Load calibration pairs from YAML file."""
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    pairs = data['calibration_pairs']
    Pc = np.array([[p['camera_frame']['x'], p['camera_frame']['y'], p['camera_frame']['z']]
                   for p in pairs])
    Pb = np.array([[p['robot_frame']['x'], p['robot_frame']['y'], p['robot_frame']['z']]
                   for p in pairs])

    return Pc, Pb, data.get('metadata', {})


def estimate_rigid_transform(Pc, Pb):
    """
    Estimate rigid transformation from camera frame to robot base frame.

    Pc: (N, 3) points in camera frame
    Pb: (N, 3) corresponding points in robot base frame

    Returns:
        R: (3, 3) rotation matrix
        t: (3,) translation vector

    The transformation is: p_base = R @ p_camera + t
    """
    assert Pc.shape == Pb.shape and Pc.shape[1] == 3

    # Compute centroids
    mu_c = Pc.mean(axis=0)
    mu_b = Pb.mean(axis=0)

    # Center the points
    X = Pc - mu_c
    Y = Pb - mu_b

    # Compute cross-covariance matrix
    H = X.T @ Y

    # SVD decomposition
    U, S, Vt = np.linalg.svd(H)

    # Compute rotation
    R = Vt.T @ U.T

    # Handle reflection case
    if np.linalg.det(R) < 0:
        Vt[-1, :] *= -1
        R = Vt.T @ U.T

    # Compute translation
    t = mu_b - R @ mu_c

    return R, t


def transform_points(Pc, R, t):
    """Transform points from camera frame to robot base frame."""
    return (Pc @ R.T) + t


def compute_errors(Pc, Pb, R, t):
    """
    Compute transformation errors.

    Returns:
        rms: Root mean square error
        errors: Per-point errors (N,)
        mean_err: Mean error
        max_err: Maximum error
    """
    pred = transform_points(Pc, R, t)
    errors = np.linalg.norm(pred - Pb, axis=1)
    rms = np.sqrt(np.mean(errors**2))
    return rms, errors, errors.mean(), errors.max()


def holdout_validation(Pc, Pb, test_ratio=0.2, seed=42):
    """
    Perform hold-out validation.

    Args:
        Pc: Camera frame points (N, 3)
        Pb: Robot base frame points (N, 3)
        test_ratio: Fraction of data to use for testing
        seed: Random seed for reproducibility

    Returns:
        Dictionary with results
    """
    np.random.seed(seed)
    N = len(Pc)
    n_test = max(1, int(N * test_ratio))

    indices = np.random.permutation(N)
    test_idx = indices[:n_test]
    train_idx = indices[n_test:]

    # Train
    R, t = estimate_rigid_transform(Pc[train_idx], Pb[train_idx])

    # Evaluate on train set
    train_rms, train_errors, train_mean, train_max = compute_errors(
        Pc[train_idx], Pb[train_idx], R, t)

    # Evaluate on test set
    test_rms, test_errors, test_mean, test_max = compute_errors(
        Pc[test_idx], Pb[test_idx], R, t)

    return {
        'R': R,
        't': t,
        'train_size': len(train_idx),
        'test_size': len(test_idx),
        'train_rms': train_rms,
        'train_mean': train_mean,
        'train_max': train_max,
        'test_rms': test_rms,
        'test_mean': test_mean,
        'test_max': test_max,
        'test_errors': test_errors,
        'test_idx': test_idx
    }


def k_fold_validation(Pc, Pb, k=5, seed=42):
    """
    Perform k-fold cross-validation.

    Returns:
        Dictionary with aggregated results
    """
    np.random.seed(seed)
    N = len(Pc)
    indices = np.random.permutation(N)
    fold_size = N // k

    all_test_errors = []
    fold_results = []

    for i in range(k):
        # Define test indices for this fold
        start = i * fold_size
        end = start + fold_size if i < k - 1 else N
        test_idx = indices[start:end]
        train_idx = np.concatenate([indices[:start], indices[end:]])

        # Train
        R, t = estimate_rigid_transform(Pc[train_idx], Pb[train_idx])

        # Evaluate on test set
        test_rms, test_errors, test_mean, test_max = compute_errors(
            Pc[test_idx], Pb[test_idx], R, t)

        all_test_errors.extend(test_errors)
        fold_results.append({
            'fold': i + 1,
            'train_size': len(train_idx),
            'test_size': len(test_idx),
            'test_rms': test_rms,
            'test_mean': test_mean,
            'test_max': test_max
        })

    all_test_errors = np.array(all_test_errors)

    return {
        'k': k,
        'fold_results': fold_results,
        'overall_rms': np.sqrt(np.mean(all_test_errors**2)),
        'overall_mean': all_test_errors.mean(),
        'overall_max': all_test_errors.max(),
        'overall_std': all_test_errors.std()
    }


def save_transformation(R, t, output_path):
    """Save transformation to YAML file."""
    data = {
        'camera_to_robot_transform': {
            'rotation_matrix': R.tolist(),
            'translation': t.tolist()
        }
    }

    with open(output_path, 'w') as f:
        yaml.dump(data, f, default_flow_style=False)

    print(f"\nTransformation saved to: {output_path}")


def main():
    parser = argparse.ArgumentParser(description='Hand-Eye Calibration')
    parser.add_argument('yaml_file', type=str, help='Path to calibration pairs YAML file')
    parser.add_argument('--test-ratio', type=float, default=0.2,
                        help='Test set ratio for hold-out validation (default: 0.2)')
    parser.add_argument('--k-fold', type=int, default=None,
                        help='Number of folds for k-fold cross-validation')
    parser.add_argument('--seed', type=int, default=42,
                        help='Random seed (default: 42)')
    parser.add_argument('--save', action='store_true',
                        help='Save the final transformation to YAML')
    args = parser.parse_args()

    # Load data
    yaml_path = Path(args.yaml_file)
    if not yaml_path.exists():
        print(f"Error: File not found: {yaml_path}")
        return

    Pc, Pb, metadata = load_calibration_pairs(yaml_path)
    N = len(Pc)

    print("=" * 60)
    print("Hand-Eye Calibration")
    print("=" * 60)
    print(f"Input file: {yaml_path.name}")
    print(f"Total pairs: {N}")
    if 'timestamp' in metadata:
        print(f"Recorded: {metadata['timestamp']}")
    print()

    # Fit on all data for final transformation
    R_full, t_full = estimate_rigid_transform(Pc, Pb)
    full_rms, full_errors, full_mean, full_max = compute_errors(Pc, Pb, R_full, t_full)

    print("Full Dataset Fit:")
    print(f"  RMS Error:  {full_rms*1000:.2f} mm")
    print(f"  Mean Error: {full_mean*1000:.2f} mm")
    print(f"  Max Error:  {full_max*1000:.2f} mm")
    print()

    # K-fold cross-validation
    if args.k_fold:
        print(f"{args.k_fold}-Fold Cross-Validation:")
        print("-" * 40)
        kfold_results = k_fold_validation(Pc, Pb, k=args.k_fold, seed=args.seed)

        for fold in kfold_results['fold_results']:
            print(f"  Fold {fold['fold']}: RMS={fold['test_rms']*1000:.2f}mm, "
                  f"Mean={fold['test_mean']*1000:.2f}mm, Max={fold['test_max']*1000:.2f}mm")

        print("-" * 40)
        print(f"  Overall RMS:  {kfold_results['overall_rms']*1000:.2f} mm")
        print(f"  Overall Mean: {kfold_results['overall_mean']*1000:.2f} mm")
        print(f"  Overall Max:  {kfold_results['overall_max']*1000:.2f} mm")
        print(f"  Overall Std:  {kfold_results['overall_std']*1000:.2f} mm")
        print()

    # Hold-out validation
    else:
        print(f"Hold-Out Validation (test ratio: {args.test_ratio}):")
        print("-" * 40)
        holdout_results = holdout_validation(Pc, Pb, test_ratio=args.test_ratio, seed=args.seed)

        print(f"  Train set: {holdout_results['train_size']} pairs")
        print(f"    RMS:  {holdout_results['train_rms']*1000:.2f} mm")
        print(f"    Mean: {holdout_results['train_mean']*1000:.2f} mm")
        print(f"    Max:  {holdout_results['train_max']*1000:.2f} mm")
        print()
        print(f"  Test set: {holdout_results['test_size']} pairs")
        print(f"    RMS:  {holdout_results['test_rms']*1000:.2f} mm")
        print(f"    Mean: {holdout_results['test_mean']*1000:.2f} mm")
        print(f"    Max:  {holdout_results['test_max']*1000:.2f} mm")
        print()

    # Print transformation
    print("Estimated Transformation (Camera -> Robot Base):")
    print("-" * 40)
    print("Rotation Matrix R:")
    for row in R_full:
        print(f"  [{row[0]:8.5f}, {row[1]:8.5f}, {row[2]:8.5f}]")
    print(f"\nTranslation t: [{t_full[0]:.5f}, {t_full[1]:.5f}, {t_full[2]:.5f}] m")
    print(f"\nUsage: p_robot = R @ p_camera + t")
    print()

    # Save if requested
    if args.save:
        output_path = yaml_path.parent / f"transform_{yaml_path.stem.replace('calibration_pairs_', '')}.yaml"
        save_transformation(R_full, t_full, output_path)

    print("=" * 60)


if __name__ == "__main__":
    main()
