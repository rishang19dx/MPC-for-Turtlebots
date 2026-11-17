**Paired Wilcoxon signed-rank, NMPC vs baselines (RMSE), Holm-corrected**

| family         | comparison                    |   n |   median diff (rmse_pos) | NMPC better in   |         p |   p (Holm) | significant   |
|:---------------|:------------------------------|----:|-------------------------:|:-----------------|----------:|-----------:|:--------------|
| circle         | NMPC vs Kanayama              |  30 |                8.014e-05 | 13%              | 0.0003128 |  0.0009383 | True          |
| circle         | NMPC vs Samson / backstepping |  30 |               -0.0001336 | 70%              | 0.0001106 |  0.0004428 | True          |
| circle         | NMPC vs Kinematic SMC         |  30 |                2.677e-05 | 37%              | 0.4045    |  0.809     | False         |
| figure8        | NMPC vs Kanayama              |  30 |               -6.509e-05 | 93%              | 8.857e-05 |  0.0004428 | True          |
| figure8        | NMPC vs Samson / backstepping |  30 |               -8.764e-05 | 97%              | 6.91e-07  |  4.146e-06 | True          |
| figure8        | NMPC vs Kinematic SMC         |  30 |                0.0001044 | 10%              | 2.049e-07 |  1.434e-06 | True          |
| lissajous      | NMPC vs Kanayama              |  30 |               -0.0002159 | 100%             | 1.863e-09 |  3.353e-08 | True          |
| lissajous      | NMPC vs Samson / backstepping |  30 |               -0.001676  | 100%             | 1.863e-09 |  3.353e-08 | True          |
| lissajous      | NMPC vs Kinematic SMC         |  30 |               -0.0004138 | 100%             | 1.863e-09 |  3.353e-08 | True          |
| point_to_point | NMPC vs Kanayama              |  30 |               -0.001053  | 100%             | 1.863e-09 |  3.353e-08 | True          |
| point_to_point | NMPC vs Samson / backstepping |  30 |               -0.001533  | 100%             | 1.863e-09 |  3.353e-08 | True          |
| point_to_point | NMPC vs Kinematic SMC         |  30 |               -0.0004044 | 100%             | 1.863e-09 |  3.353e-08 | True          |
| random_spline  | NMPC vs Kanayama              |  30 |               -0.0004421 | 100%             | 1.863e-09 |  3.353e-08 | True          |
| random_spline  | NMPC vs Samson / backstepping |  30 |               -0.0005258 | 100%             | 1.863e-09 |  3.353e-08 | True          |
| random_spline  | NMPC vs Kinematic SMC         |  30 |               -8.143e-05 | 97%              | 3.725e-09 |  3.353e-08 | True          |
| sinusoid       | NMPC vs Kanayama              |  30 |               -0.0002388 | 100%             | 1.863e-09 |  3.353e-08 | True          |
| sinusoid       | NMPC vs Samson / backstepping |  30 |               -0.0002769 | 100%             | 1.863e-09 |  3.353e-08 | True          |
| sinusoid       | NMPC vs Kinematic SMC         |  30 |                0.0001267 | 40%              | 0.8872    |  0.8872    | False         |
