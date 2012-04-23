#pragma once
//total time: 10.100000
#define N_MC_FRAMES 101
#define N_MC_MARKERS 15
#define MC_SAMPLERATE 10
float markers[N_MC_FRAMES][N_MC_MARKERS][3]={
{ {-8.36f,63.65f,-7.92f}, {-4.72f,62.53f,-19.39f}, {0.90f,72.17f,-10.90f}, {16.67f,78.32f,-10.80f}, {27.48f,76.38f,-5.42f}, {6.35f,43.00f,-32.85f}, {0.10f,53.82f,-31.88f}, {14.48f,53.67f,-4.68f}, {11.50f,49.55f,-16.25f}, {17.68f,25.48f,-22.80f}, {8.62f,30.09f,-4.97f}, {23.96f,19.30f,-2.80f}, {22.88f,5.34f,-22.07f}, {26.04f,11.28f,-3.22f}, {17.16f,1.22f,-21.67f},  },
{ {-10.71f,57.78f,-7.23f}, {-5.99f,55.58f,-17.41f}, {-1.32f,66.45f,-9.80f}, {13.43f,73.95f,-10.59f}, {24.93f,74.74f,-6.70f}, {-1.77f,33.15f,-32.16f}, {-4.47f,44.30f,-28.49f}, {13.80f,49.97f,-3.76f}, {11.28f,45.44f,-15.34f}, {12.76f,22.94f,-22.65f}, {6.02f,27.79f,-3.67f}, {6.12f,8.69f,-1.25f}, {22.82f,4.90f,-21.93f}, {-0.20f,3.92f,-2.64f}, {17.23f,1.20f,-21.60f},  },
{ {-12.75f,51.56f,-6.83f}, {-7.25f,50.29f,-16.42f}, {-3.78f,60.72f,-8.56f}, {10.09f,69.07f,-8.38f}, {20.57f,73.04f,-3.84f}, {-10.09f,26.56f,-28.72f}, {-8.07f,37.80f,-26.04f}, {10.93f,44.79f,-1.75f}, {9.80f,41.54f,-13.98f}, {9.22f,20.35f,-22.23f}, {1.34f,23.93f,-1.85f}, {3.31f,5.57f,2.37f}, {22.95f,5.36f,-21.85f}, {-2.50f,1.62f,-0.47f}, {17.22f,1.14f,-21.78f},  },
{ {-15.10f,48.38f,-7.83f}, {-7.25f,50.30f,-17.17f}, {-6.75f,57.92f,-8.20f}, {6.67f,65.24f,-1.29f}, {14.06f,68.19f,7.73f}, {-8.47f,23.64f,-28.56f}, {-7.32f,34.79f,-25.63f}, {8.11f,43.25f,0.23f}, {8.48f,42.33f,-12.51f}, {6.84f,19.51f,-20.62f}, {-1.77f,22.95f,-0.24f}, {3.48f,4.79f,2.28f}, {21.99f,6.21f,-22.56f}, {-2.42f,1.64f,-0.31f}, {16.79f,1.16f,-21.96f},  },
{ {-16.97f,49.16f,-11.04f}, {-7.23f,50.36f,-18.95f}, {-10.05f,58.10f,-7.72f}, {-3.48f,59.29f,8.64f}, {-3.72f,56.58f,19.75f}, {6.10f,29.22f,-31.70f}, {1.33f,39.06f,-28.05f}, {3.97f,44.89f,1.93f}, {7.30f,44.04f,-10.52f}, {7.95f,20.77f,-17.73f}, {-3.02f,22.48f,0.95f}, {3.52f,4.57f,2.59f}, {21.98f,6.32f,-21.17f}, {-2.27f,1.72f,-0.01f}, {16.78f,1.45f,-21.39f},  },
{ {-17.09f,52.35f,-15.55f}, {-4.63f,55.68f,-20.53f}, {-13.60f,58.98f,-9.34f}, {-20.44f,50.40f,4.05f}, {-27.50f,42.81f,7.07f}, {21.67f,49.93f,-27.64f}, {11.66f,54.67f,-24.95f}, {-1.47f,46.50f,2.39f}, {6.23f,46.42f,-8.22f}, {11.63f,24.17f,-15.45f}, {-3.93f,22.22f,1.55f}, {3.28f,4.46f,2.92f}, {21.14f,6.05f,-17.83f}, {-2.20f,1.67f,-0.17f}, {16.06f,2.13f,-20.44f},  },
{ {-14.70f,56.85f,-19.67f}, {-2.24f,66.65f,-18.19f}, {-14.58f,61.84f,-12.76f}, {-22.72f,47.18f,-10.59f}, {-26.05f,39.35f,-18.08f}, {22.62f,76.19f,-11.46f}, {11.37f,73.93f,-11.40f}, {-6.28f,47.39f,-0.55f}, {5.26f,49.20f,-5.79f}, {13.31f,26.24f,-9.75f}, {-5.46f,22.53f,0.24f}, {1.10f,5.09f,4.56f}, {17.86f,6.27f,-8.77f}, {-2.19f,1.60f,-0.45f}, {15.83f,2.53f,-14.56f},  },
{ {-9.25f,63.89f,-22.95f}, {-3.84f,74.75f,-11.70f}, {-12.66f,64.49f,-16.37f}, {-14.71f,46.88f,-16.43f}, {-7.06f,42.43f,-24.17f}, {9.86f,90.63f,1.28f}, {1.30f,82.98f,2.65f}, {-7.47f,48.97f,-6.36f}, {4.20f,53.04f,-2.45f}, {11.89f,26.81f,-0.70f}, {-5.95f,23.03f,-3.67f}, {-5.63f,5.67f,4.22f}, {11.16f,6.36f,1.77f}, {-2.52f,1.69f,-0.72f}, {13.78f,2.60f,-3.79f},  },
{ {-0.62f,72.55f,-20.30f}, {-8.32f,77.59f,-6.61f}, {-6.96f,67.33f,-17.56f}, {-5.10f,49.46f,-15.39f}, {6.60f,50.34f,-14.76f}, {-5.06f,91.59f,1.62f}, {-11.47f,84.64f,8.32f}, {-2.29f,50.82f,-9.14f}, {-0.50f,54.96f,3.44f}, {4.47f,26.92f,9.63f}, {-4.14f,23.24f,-5.76f}, {-8.59f,4.97f,-1.65f}, {2.63f,6.32f,8.18f}, {-2.40f,1.64f,-0.53f}, {8.58f,2.45f,6.69f},  },
{ {2.50f,79.08f,-8.99f}, {-13.71f,76.47f,-5.26f}, {-1.71f,70.18f,-13.65f}, {5.49f,54.18f,-11.78f}, {13.57f,61.61f,-7.58f}, {-20.73f,85.02f,1.61f}, {-27.39f,76.11f,3.82f}, {0.69f,51.56f,-3.21f}, {-9.03f,53.63f,5.71f}, {-8.20f,26.31f,13.07f}, {-2.25f,23.09f,-5.43f}, {-6.34f,4.37f,-5.19f}, {-5.69f,5.93f,10.35f}, {-2.63f,1.69f,0.01f}, {-0.32f,2.41f,13.86f},  },
{ {-4.36f,81.13f,1.12f}, {-17.40f,73.61f,-7.36f}, {-2.44f,73.04f,-7.98f}, {13.42f,66.58f,-8.18f}, {14.75f,77.28f,-4.07f}, {-36.41f,74.00f,-6.05f}, {-31.47f,64.99f,-11.21f}, {-2.56f,51.00f,3.96f}, {-15.68f,51.37f,4.38f}, {-17.54f,25.28f,11.19f}, {-1.84f,23.10f,-2.45f}, {-5.14f,4.21f,-5.56f}, {-11.76f,5.62f,9.33f}, {-3.03f,1.76f,0.54f}, {-10.33f,2.41f,15.62f},  },
{ {-13.41f,81.49f,4.22f}, {-18.60f,71.05f,-8.81f}, {-7.13f,74.82f,-3.73f}, {8.65f,79.91f,-2.45f}, {4.54f,90.62f,-2.37f}, {-32.94f,57.60f,-22.06f}, {-22.45f,56.69f,-17.97f}, {-7.10f,51.20f,8.30f}, {-19.59f,51.70f,4.36f}, {-23.39f,24.60f,8.10f}, {-2.84f,23.18f,0.27f}, {-2.63f,4.72f,-5.17f}, {-17.91f,4.67f,9.92f}, {-2.45f,1.73f,1.00f}, {-21.21f,2.63f,15.70f},  },
{ {-19.99f,80.74f,4.65f}, {-19.70f,69.22f,-9.56f}, {-12.64f,74.83f,-0.35f}, {-0.95f,83.23f,7.69f}, {-7.41f,91.82f,3.19f}, {-20.74f,49.49f,-24.93f}, {-15.07f,53.73f,-16.50f}, {-13.24f,51.43f,10.70f}, {-24.21f,49.83f,3.65f}, {-26.40f,22.79f,4.60f}, {-4.62f,23.76f,3.15f}, {-1.19f,6.22f,-4.10f}, {-22.88f,3.59f,10.80f}, {-2.10f,1.71f,1.15f}, {-30.48f,2.41f,12.62f},  },
{ {-26.35f,80.75f,4.71f}, {-21.70f,70.32f,-8.82f}, {-17.14f,74.38f,4.29f}, {-10.03f,73.50f,19.40f}, {-14.93f,83.41f,16.09f}, {-21.84f,57.09f,-28.77f}, {-17.49f,56.38f,-18.35f}, {-21.93f,50.07f,11.13f}, {-29.60f,49.03f,0.32f}, {-29.05f,21.61f,4.05f}, {-10.43f,24.06f,11.85f}, {0.62f,10.05f,4.31f}, {-23.90f,3.34f,11.69f}, {-0.88f,2.96f,6.04f}, {-30.49f,2.26f,13.01f},  },
{ {-31.38f,80.15f,5.82f}, {-24.64f,72.69f,-5.96f}, {-22.04f,70.16f,9.89f}, {-19.57f,55.65f,18.68f}, {-24.79f,63.87f,24.90f}, {-28.30f,76.58f,-28.29f}, {-22.88f,68.45f,-22.36f}, {-27.03f,48.67f,9.53f}, {-32.13f,47.61f,-2.76f}, {-30.85f,20.68f,2.57f}, {-20.89f,25.06f,18.07f}, {-9.89f,9.71f,21.03f}, {-24.05f,3.87f,12.49f}, {-12.64f,3.79f,24.69f}, {-30.59f,2.24f,13.21f},  },
{ {-35.88f,78.33f,6.53f}, {-29.44f,73.24f,-2.78f}, {-27.21f,66.12f,10.13f}, {-23.31f,48.45f,10.65f}, {-30.80f,45.93f,18.89f}, {-34.67f,91.90f,-14.57f}, {-28.33f,82.37f,-16.27f}, {-31.83f,47.63f,6.60f}, {-31.42f,48.17f,-6.46f}, {-31.19f,19.65f,1.40f}, {-37.07f,25.82f,16.90f}, {-33.31f,9.35f,25.62f}, {-24.06f,4.05f,12.76f}, {-38.43f,3.77f,26.79f}, {-30.68f,2.26f,13.29f},  },
{ {-40.62f,77.14f,4.60f}, {-33.34f,72.49f,-2.66f}, {-31.47f,65.18f,8.82f}, {-24.42f,48.27f,8.75f}, {-30.83f,42.43f,16.02f}, {-44.71f,90.26f,-2.47f}, {-38.12f,84.37f,-10.63f}, {-34.98f,47.87f,3.87f}, {-31.95f,48.57f,-8.83f}, {-31.59f,19.27f,-0.27f}, {-46.63f,26.74f,6.53f}, {-50.38f,8.27f,10.46f}, {-24.40f,4.58f,12.41f}, {-55.68f,3.31f,6.84f}, {-30.85f,2.29f,13.18f},  },
{ {-44.99f,76.07f,2.37f}, {-35.43f,71.59f,-4.47f}, {-34.82f,65.08f,6.88f}, {-24.50f,50.60f,10.11f}, {-30.54f,46.16f,18.55f}, {-57.88f,78.63f,-2.18f}, {-49.57f,78.56f,-11.14f}, {-37.96f,47.25f,1.59f}, {-34.12f,47.82f,-10.79f}, {-32.00f,18.45f,-2.19f}, {-49.03f,26.41f,-3.56f}, {-53.21f,7.42f,-5.62f}, {-24.77f,5.15f,12.02f}, {-55.48f,3.25f,-12.49f}, {-30.95f,2.21f,12.82f},  },
{ {-49.66f,73.40f,-0.39f}, {-36.07f,69.25f,-7.96f}, {-39.90f,63.12f,4.15f}, {-26.90f,52.09f,9.39f}, {-32.48f,47.91f,18.41f}, {-66.94f,62.77f,-9.94f}, {-57.46f,67.16f,-16.70f}, {-41.40f,44.36f,-1.48f}, {-36.14f,45.12f,-13.17f}, {-31.27f,16.14f,-4.50f}, {-50.39f,23.86f,-10.77f}, {-50.94f,4.96f,-8.73f}, {-25.18f,6.14f,12.65f}, {-52.74f,1.16f,-15.36f}, {-31.00f,2.17f,12.40f},  },
{ {-54.12f,70.57f,-4.61f}, {-36.51f,66.13f,-12.23f}, {-45.44f,61.18f,0.72f}, {-32.87f,51.80f,9.40f}, {-38.96f,47.14f,17.83f}, {-65.17f,53.44f,-23.25f}, {-56.33f,56.64f,-25.79f}, {-43.36f,42.21f,-3.09f}, {-37.03f,42.66f,-14.31f}, {-27.39f,15.27f,-7.60f}, {-51.77f,23.22f,-12.83f}, {-50.78f,4.59f,-9.05f}, {-25.46f,8.58f,11.95f}, {-52.77f,1.15f,-15.48f}, {-28.56f,2.65f,10.05f},  },
{ {-56.69f,69.05f,-10.72f}, {-36.78f,62.96f,-16.43f}, {-50.11f,60.29f,-2.81f}, {-41.56f,52.45f,10.81f}, {-48.94f,47.92f,18.15f}, {-46.25f,51.31f,-35.54f}, {-44.77f,52.83f,-32.03f}, {-44.71f,40.73f,-4.39f}, {-36.20f,41.84f,-13.92f}, {-21.09f,18.27f,-9.69f}, {-52.11f,22.91f,-14.04f}, {-50.69f,4.50f,-9.27f}, {-21.09f,9.19f,8.74f}, {-52.76f,1.16f,-15.51f}, {-17.51f,4.56f,5.82f},  },
{ {-55.56f,67.54f,-17.87f}, {-36.92f,60.45f,-19.70f}, {-52.51f,59.18f,-7.39f}, {-50.04f,53.59f,9.09f}, {-57.48f,49.84f,16.42f}, {-23.00f,53.18f,-37.05f}, {-30.08f,56.54f,-28.11f}, {-44.63f,39.21f,-5.78f}, {-34.67f,41.32f,-13.38f}, {-15.07f,21.49f,-7.79f}, {-52.24f,22.72f,-14.92f}, {-50.64f,4.53f,-9.36f}, {-9.30f,5.86f,4.65f}, {-52.78f,1.17f,-15.52f}, {-4.26f,2.93f,0.47f},  },
{ {-49.10f,65.76f,-24.02f}, {-36.97f,59.31f,-21.16f}, {-50.36f,57.69f,-12.57f}, {-52.16f,54.11f,3.85f}, {-59.02f,52.57f,12.27f}, {-11.35f,65.17f,-15.24f}, {-22.74f,62.22f,-15.09f}, {-42.70f,37.57f,-6.97f}, {-31.09f,39.99f,-12.31f}, {-10.45f,23.35f,-4.41f}, {-51.91f,23.03f,-14.77f}, {-50.61f,4.61f,-9.30f}, {1.05f,6.35f,0.92f}, {-52.82f,1.17f,-15.51f}, {5.34f,3.43f,-4.50f},  },
{ {-38.53f,63.67f,-27.04f}, {-29.50f,59.20f,-19.05f}, {-43.15f,55.85f,-16.36f}, {-48.51f,53.68f,-0.83f}, {-55.96f,53.67f,7.11f}, {-17.47f,68.74f,3.38f}, {-23.95f,63.29f,-4.57f}, {-38.37f,36.38f,-7.07f}, {-25.63f,37.99f,-9.61f}, {-6.48f,23.66f,-3.55f}, {-49.64f,23.47f,-14.09f}, {-50.62f,4.69f,-9.10f}, {2.90f,5.50f,2.84f}, {-52.95f,1.22f,-15.48f}, {7.04f,2.03f,-2.50f},  },
{ {-26.01f,62.49f,-26.31f}, {-20.29f,58.23f,-15.37f}, {-33.50f,54.21f,-18.07f}, {-43.43f,51.84f,-5.09f}, {-52.83f,51.76f,0.36f}, {-21.42f,64.68f,11.03f}, {-22.51f,60.54f,0.51f}, {-32.25f,35.50f,-7.07f}, {-19.43f,36.23f,-5.99f}, {-2.29f,23.36f,-2.44f}, {-45.52f,23.43f,-13.50f}, {-50.17f,4.81f,-8.99f}, {3.06f,3.77f,3.40f}, {-52.73f,1.39f,-15.40f}, {7.42f,1.78f,-2.30f},  },
{ {-13.64f,63.25f,-22.36f}, {-10.83f,57.74f,-9.81f}, {-22.52f,53.93f,-18.21f}, {-37.83f,49.25f,-12.81f}, {-48.42f,47.25f,-13.65f}, {-17.03f,59.08f,16.61f}, {-16.67f,57.46f,5.35f}, {-25.68f,35.27f,-7.27f}, {-14.04f,36.09f,-2.05f}, {1.03f,23.89f,-1.44f}, {-40.02f,22.00f,-14.11f}, {-49.01f,5.07f,-9.36f}, {3.33f,3.67f,3.55f}, {-51.85f,1.81f,-15.62f}, {7.75f,2.06f,-1.93f},  },
{ {-3.07f,64.10f,-16.80f}, {-3.61f,57.41f,-3.27f}, {-11.33f,54.35f,-16.72f}, {-25.56f,46.06f,-22.10f}, {-31.51f,41.06f,-29.56f}, {-12.83f,50.35f,21.61f}, {-12.08f,53.49f,10.45f}, {-19.39f,35.87f,-8.25f}, {-10.42f,36.18f,0.74f}, {3.18f,24.05f,0.34f}, {-33.68f,18.50f,-14.60f}, {-47.31f,5.24f,-10.33f}, {3.14f,3.62f,4.04f}, {-50.32f,2.10f,-16.41f}, {8.39f,2.58f,-0.53f},  },
{ {4.54f,63.20f,-11.22f}, {-0.89f,56.28f,2.35f}, {-2.40f,53.99f,-13.42f}, {-7.59f,41.97f,-25.42f}, {-5.78f,35.15f,-33.60f}, {-13.51f,39.74f,20.35f}, {-12.07f,47.83f,11.97f}, {-13.65f,34.92f,-8.93f}, {-8.07f,35.90f,2.46f}, {5.11f,23.89f,2.12f}, {-28.15f,15.12f,-14.20f}, {-45.00f,5.99f,-12.32f}, {3.31f,3.38f,4.42f}, {-47.72f,2.41f,-18.05f}, {8.86f,2.19f,0.24f},  },
{ {9.60f,60.03f,-6.77f}, {-1.17f,54.85f,5.47f}, {4.40f,52.78f,-8.66f}, {10.83f,40.22f,-18.93f}, {18.38f,33.19f,-23.10f}, {-17.47f,33.70f,13.32f}, {-14.28f,44.12f,8.81f}, {-9.98f,33.76f,-9.23f}, {-6.65f,35.35f,2.84f}, {7.03f,23.53f,3.42f}, {-25.04f,13.80f,-14.98f}, {-42.86f,6.43f,-15.92f}, {3.38f,3.23f,4.54f}, {-46.37f,1.34f,-19.52f}, {9.08f,2.05f,0.62f},  },
{ {12.76f,58.08f,-3.35f}, {-1.38f,54.39f,6.47f}, {7.83f,52.00f,-4.52f}, {19.08f,41.35f,-9.43f}, {28.97f,35.11f,-9.69f}, {-20.66f,34.46f,8.62f}, {-15.69f,44.69f,5.32f}, {-6.77f,33.40f,-9.39f}, {-5.90f,35.76f,2.67f}, {7.61f,23.34f,3.96f}, {-24.29f,14.51f,-16.08f}, {-41.89f,6.52f,-17.62f}, {3.33f,3.26f,4.36f}, {-45.74f,0.45f,-20.27f}, {9.19f,1.95f,0.66f},  },
{ {14.27f,58.35f,-1.37f}, {-0.78f,54.75f,7.09f}, {9.43f,52.47f,-3.09f}, {20.92f,42.17f,-6.67f}, {30.90f,35.98f,-5.57f}, {-21.86f,36.77f,7.38f}, {-15.70f,46.39f,4.66f}, {-5.38f,34.66f,-9.97f}, {-5.70f,36.60f,2.19f}, {7.09f,23.44f,4.19f}, {-24.50f,15.83f,-16.54f}, {-41.61f,6.79f,-18.59f}, {3.21f,3.30f,4.21f}, {-45.27f,0.00f,-20.79f}, {9.20f,1.92f,0.68f},  },
{ {14.59f,59.34f,-1.45f}, {-0.22f,55.44f,7.17f}, {9.51f,53.71f,-3.52f}, {21.06f,43.64f,-7.04f}, {30.98f,37.43f,-5.60f}, {-21.40f,37.61f,7.35f}, {-15.33f,47.40f,4.96f}, {-5.95f,36.31f,-10.68f}, {-5.67f,37.76f,1.54f}, {6.44f,23.66f,3.17f}, {-25.90f,18.02f,-16.77f}, {-41.94f,7.24f,-19.07f}, {3.14f,3.40f,3.97f}, {-45.18f,0.03f,-21.07f}, {9.05f,1.88f,0.49f},  },
{ {13.33f,61.52f,-3.04f}, {0.04f,56.30f,6.77f}, {7.73f,56.18f,-5.41f}, {18.65f,45.29f,-9.77f}, {28.45f,39.12f,-7.08f}, {-18.84f,37.10f,9.00f}, {-15.38f,48.02f,6.26f}, {-7.94f,38.14f,-11.55f}, {-5.96f,38.66f,0.77f}, {5.74f,23.86f,2.24f}, {-27.78f,19.69f,-16.58f}, {-43.85f,8.53f,-17.25f}, {3.12f,3.52f,3.63f}, {-45.73f,1.59f,-20.87f}, {9.00f,1.87f,0.22f},  },
{ {8.92f,65.27f,-5.00f}, {-1.49f,57.67f,6.12f}, {2.79f,59.77f,-7.99f}, {6.85f,45.70f,-16.45f}, {15.63f,41.78f,-10.80f}, {-7.52f,38.92f,7.45f}, {-15.49f,47.64f,8.36f}, {-11.23f,40.07f,-12.48f}, {-7.19f,39.66f,-0.44f}, {4.62f,24.01f,1.14f}, {-28.99f,20.11f,-16.29f}, {-44.96f,9.01f,-12.63f}, {3.15f,3.57f,3.64f}, {-45.22f,2.49f,-16.89f}, {9.09f,1.87f,0.27f},  },
{ {1.48f,68.90f,-7.63f}, {-3.94f,59.74f,3.19f}, {-8.00f,64.63f,-11.25f}, {-17.14f,53.29f,-19.69f}, {-7.64f,48.87f,-17.08f}, {3.47f,43.21f,0.94f}, {-5.86f,43.22f,7.32f}, {-16.07f,41.58f,-13.53f}, {-9.67f,40.74f,-2.17f}, {2.63f,24.07f,0.18f}, {-31.77f,19.65f,-15.92f}, {-45.83f,6.98f,-10.04f}, {3.07f,3.58f,3.75f}, {-45.72f,1.62f,-15.32f}, {9.15f,1.88f,0.38f},  },
{ {-6.72f,71.08f,-11.14f}, {-7.29f,62.61f,-4.31f}, {-20.14f,67.72f,-10.02f}, {-35.11f,63.60f,-8.76f}, {-29.70f,58.33f,-15.97f}, {10.35f,42.70f,-3.37f}, {2.10f,48.43f,-0.34f}, {-23.64f,42.71f,-14.65f}, {-13.45f,41.94f,-6.37f}, {-0.39f,23.91f,-1.20f}, {-35.47f,19.21f,-15.75f}, {-47.19f,5.11f,-8.88f}, {2.77f,3.71f,3.63f}, {-47.15f,1.48f,-15.01f}, {8.64f,2.02f,-0.23f},  },
{ {-13.02f,72.30f,-14.12f}, {-12.17f,64.03f,-7.78f}, {-26.46f,68.37f,-9.57f}, {-39.42f,62.70f,-3.28f}, {-35.76f,57.34f,-11.68f}, {7.72f,46.21f,-5.82f}, {-1.05f,51.95f,-3.34f}, {-29.06f,43.27f,-13.51f}, {-16.77f,43.28f,-8.94f}, {-3.11f,23.52f,-1.35f}, {-37.17f,19.77f,-16.76f}, {-47.49f,5.10f,-9.05f}, {2.49f,3.81f,3.36f}, {-47.22f,1.34f,-15.12f}, {8.06f,2.04f,-1.04f},  },
{ {-15.68f,73.47f,-13.59f}, {-14.74f,65.09f,-7.18f}, {-29.15f,68.36f,-10.29f}, {-42.57f,60.49f,-7.73f}, {-35.78f,56.02f,-14.70f}, {5.35f,47.68f,-3.90f}, {-3.59f,53.48f,-1.67f}, {-30.20f,43.07f,-13.33f}, {-17.86f,43.61f,-8.65f}, {-4.52f,23.31f,-0.60f}, {-38.60f,20.39f,-16.64f}, {-48.07f,5.00f,-9.07f}, {2.26f,3.79f,3.27f}, {-47.41f,1.39f,-15.14f}, {8.00f,2.10f,-1.01f},  },
{ {-15.53f,74.00f,-12.13f}, {-15.16f,65.51f,-4.74f}, {-28.89f,68.20f,-11.48f}, {-40.45f,57.36f,-14.95f}, {-30.57f,54.97f,-18.31f}, {2.60f,48.48f,5.68f}, {-6.48f,54.44f,5.04f}, {-29.59f,42.94f,-12.97f}, {-17.53f,43.57f,-7.87f}, {-4.69f,23.32f,0.43f}, {-38.26f,20.48f,-16.01f}, {-48.03f,4.93f,-9.09f}, {2.18f,3.69f,3.51f}, {-47.32f,1.43f,-15.11f}, {8.06f,2.25f,-0.64f},  },
{ {-12.70f,74.11f,-10.18f}, {-14.43f,65.44f,-0.93f}, {-24.54f,67.34f,-13.46f}, {-27.86f,52.65f,-20.88f}, {-17.22f,53.17f,-19.18f}, {-6.04f,48.72f,17.50f}, {-13.36f,55.28f,13.27f}, {-26.85f,43.00f,-13.16f}, {-15.93f,43.16f,-6.04f}, {-3.37f,23.81f,1.35f}, {-37.49f,20.49f,-15.08f}, {-47.94f,4.95f,-8.98f}, {1.86f,3.73f,3.70f}, {-47.20f,1.63f,-15.02f}, {8.27f,2.83f,0.59f},  },
{ {-7.33f,73.30f,-7.86f}, {-13.10f,64.18f,2.69f}, {-14.65f,65.34f,-13.52f}, {-9.10f,50.80f,-19.75f}, {0.99f,51.43f,-14.50f}, {-19.55f,47.19f,20.59f}, {-23.10f,54.77f,13.94f}, {-22.13f,42.08f,-12.99f}, {-13.57f,41.76f,-3.18f}, {-1.60f,24.15f,3.33f}, {-36.15f,20.17f,-14.06f}, {-47.86f,5.11f,-9.03f}, {1.05f,3.56f,4.31f}, {-47.15f,1.54f,-15.14f}, {7.92f,3.09f,4.01f},  },
{ {-1.53f,72.28f,-4.56f}, {-11.98f,62.92f,5.29f}, {-3.95f,63.15f,-8.54f}, {6.61f,50.99f,-11.51f}, {16.40f,44.93f,-9.17f}, {-28.49f,45.12f,11.93f}, {-27.43f,53.89f,6.07f}, {-14.78f,40.54f,-11.61f}, {-10.62f,40.99f,0.54f}, {0.81f,24.11f,5.60f}, {-33.92f,20.74f,-11.74f}, {-47.35f,6.54f,-9.46f}, {0.97f,3.35f,4.67f}, {-47.27f,1.59f,-15.35f}, {7.55f,2.68f,5.59f},  },
{ {2.92f,73.57f,-1.29f}, {-9.89f,63.81f,6.69f}, {2.25f,64.11f,-4.27f}, {10.90f,49.53f,-6.35f}, {20.02f,43.86f,-1.52f}, {-25.23f,44.18f,5.64f}, {-22.74f,53.10f,0.66f}, {-6.85f,41.11f,-8.84f}, {-6.70f,42.00f,3.85f}, {2.85f,23.36f,8.49f}, {-26.16f,21.11f,-13.65f}, {-41.64f,9.26f,-13.46f}, {0.92f,3.20f,4.73f}, {-43.22f,2.32f,-16.79f}, {7.16f,2.26f,6.58f},  },
{ {5.20f,76.15f,-0.06f}, {-8.39f,66.00f,6.52f}, {5.49f,66.06f,-2.02f}, {13.27f,50.74f,-0.09f}, {20.86f,49.25f,8.34f}, {-15.30f,46.46f,15.56f}, {-16.57f,50.36f,6.15f}, {-0.57f,42.29f,-5.75f}, {-4.11f,43.93f,6.56f}, {3.71f,22.72f,10.02f}, {-12.76f,19.10f,-13.98f}, {-29.62f,9.71f,-15.82f}, {0.96f,3.12f,4.78f}, {-30.56f,2.62f,-18.15f}, {7.15f,2.16f,6.75f},  },
{ {5.97f,78.34f,0.14f}, {-7.60f,69.07f,5.60f}, {5.75f,69.68f,-1.88f}, {18.66f,60.88f,4.53f}, {24.34f,67.16f,12.28f}, {-7.32f,64.57f,24.67f}, {-11.15f,58.10f,17.59f}, {4.01f,43.87f,-2.39f}, {-2.92f,45.66f,8.62f}, {3.49f,22.13f,11.35f}, {1.26f,20.15f,-11.23f}, {-12.27f,7.97f,-17.26f}, {1.10f,3.01f,4.62f}, {-10.17f,2.15f,-21.23f}, {7.31f,2.04f,6.81f},  },
{ {7.01f,81.15f,0.60f}, {-4.77f,74.54f,4.00f}, {4.97f,73.96f,-1.85f}, {19.57f,76.84f,3.32f}, {25.25f,85.96f,7.31f}, {-5.27f,85.40f,24.80f}, {-7.83f,75.38f,19.81f}, {7.43f,48.12f,1.73f}, {-2.65f,49.71f,10.32f}, {2.07f,22.36f,11.01f}, {10.47f,23.87f,-8.11f}, {6.08f,7.15f,-16.61f}, {1.20f,3.00f,4.24f}, {11.24f,2.14f,-18.81f}, {7.37f,2.06f,6.74f},  },
{ {8.26f,84.22f,1.38f}, {-2.10f,79.11f,3.27f}, {4.86f,77.84f,-0.97f}, {16.82f,85.47f,0.59f}, {22.22f,95.58f,1.30f}, {-4.32f,92.16f,22.22f}, {-4.81f,87.32f,14.80f}, {9.39f,53.83f,5.20f}, {-1.94f,55.34f,11.52f}, {-0.27f,23.71f,7.89f}, {15.41f,27.59f,-5.30f}, {18.76f,9.46f,-11.08f}, {2.17f,4.01f,2.20f}, {23.35f,2.87f,-10.18f}, {7.57f,2.09f,6.10f},  },
{ {9.44f,86.44f,2.40f}, {-0.84f,81.35f,3.04f}, {6.01f,80.06f,-0.26f}, {17.44f,87.91f,-0.46f}, {21.87f,98.76f,-0.35f}, {-2.87f,95.55f,20.33f}, {-4.29f,89.54f,13.57f}, {10.45f,56.43f,7.27f}, {-1.64f,58.02f,12.80f}, {-1.18f,25.71f,5.87f}, {18.28f,29.58f,-0.94f}, {23.13f,11.19f,-3.76f}, {3.24f,5.80f,2.27f}, {26.19f,4.07f,-0.98f}, {7.74f,2.08f,6.49f},  },
{ {10.99f,86.98f,3.48f}, {0.85f,81.56f,3.38f}, {8.04f,79.99f,0.17f}, {22.03f,85.16f,1.21f}, {25.94f,96.11f,2.08f}, {-1.88f,96.93f,19.35f}, {-5.20f,89.04f,15.03f}, {10.97f,56.47f,7.47f}, {-1.45f,57.99f,11.91f}, {-0.40f,25.99f,6.78f}, {18.97f,29.56f,5.84f}, {22.38f,10.95f,2.43f}, {3.75f,6.21f,2.37f}, {24.46f,3.64f,5.33f}, {7.79f,2.07f,6.65f},  },
{ {13.33f,85.11f,4.79f}, {1.97f,79.25f,7.05f}, {12.51f,75.92f,0.79f}, {26.92f,69.50f,3.13f}, {27.63f,80.05f,5.82f}, {-3.20f,84.18f,19.14f}, {-9.66f,75.89f,16.86f}, {11.96f,52.54f,5.52f}, {-0.22f,53.73f,10.81f}, {0.47f,24.25f,9.19f}, {18.02f,30.73f,13.33f}, {20.16f,13.40f,5.75f}, {2.67f,4.83f,2.00f}, {21.53f,5.43f,6.70f}, {7.30f,1.98f,6.64f},  },
{ {16.22f,80.47f,8.00f}, {2.06f,71.59f,10.78f}, {17.18f,67.35f,4.09f}, {18.50f,51.69f,0.84f}, {24.19f,58.29f,6.38f}, {-5.19f,64.13f,15.56f}, {-8.38f,59.67f,6.82f}, {13.52f,47.43f,4.90f}, {1.38f,47.14f,9.92f}, {2.08f,21.22f,10.22f}, {16.88f,31.69f,17.96f}, {19.38f,15.07f,9.72f}, {3.05f,2.95f,0.81f}, {20.17f,6.87f,10.04f}, {7.27f,1.94f,6.23f},  },
{ {17.56f,77.48f,11.73f}, {2.97f,69.69f,11.76f}, {18.30f,65.38f,6.94f}, {21.28f,50.67f,1.08f}, {25.61f,56.87f,8.32f}, {-7.93f,63.69f,15.30f}, {-9.24f,61.05f,5.14f}, {14.60f,45.21f,6.27f}, {1.82f,44.75f,9.44f}, {2.12f,20.50f,12.81f}, {17.40f,29.96f,19.82f}, {22.12f,12.19f,15.07f}, {3.08f,3.63f,1.05f}, {23.56f,4.83f,18.22f}, {7.20f,1.94f,6.43f},  },
{ {16.42f,76.50f,14.46f}, {3.24f,68.38f,13.00f}, {17.72f,67.08f,9.50f}, {29.02f,59.71f,1.54f}, {31.28f,62.96f,11.79f}, {-15.04f,64.13f,15.63f}, {-9.81f,62.85f,5.37f}, {16.60f,44.09f,8.16f}, {3.56f,43.60f,9.72f}, {0.99f,20.56f,14.52f}, {20.12f,27.15f,20.72f}, {27.31f,9.52f,20.49f}, {2.82f,4.68f,1.66f}, {27.43f,3.63f,26.15f}, {7.01f,1.96f,6.71f},  },
{ {14.83f,73.56f,16.87f}, {2.49f,63.67f,14.77f}, {16.06f,66.49f,13.65f}, {30.02f,69.68f,8.86f}, {38.24f,68.06f,17.57f}, {-20.49f,55.66f,6.54f}, {-8.84f,57.82f,5.18f}, {19.45f,41.61f,9.84f}, {6.38f,41.29f,10.67f}, {2.75f,20.05f,17.51f}, {22.53f,24.48f,22.18f}, {31.15f,7.59f,20.23f}, {2.46f,5.68f,2.98f}, {29.65f,2.50f,25.91f}, {7.00f,1.92f,6.87f},  },
{ {14.25f,70.11f,18.43f}, {2.87f,58.42f,14.32f}, {15.42f,63.70f,16.28f}, {27.89f,70.78f,18.58f}, {35.11f,74.53f,27.73f}, {-12.53f,43.82f,-1.69f}, {-3.06f,49.94f,1.49f}, {21.92f,38.89f,10.58f}, {9.01f,38.28f,12.34f}, {5.51f,18.33f,20.36f}, {24.75f,23.19f,23.75f}, {31.69f,6.02f,19.74f}, {2.57f,6.37f,4.33f}, {30.03f,2.35f,25.85f}, {7.15f,1.88f,6.96f},  },
{ {14.68f,69.08f,17.92f}, {3.95f,55.72f,13.08f}, {15.11f,62.80f,15.95f}, {22.15f,71.59f,24.06f}, {18.96f,71.21f,35.78f}, {-3.80f,33.83f,-0.72f}, {2.38f,43.31f,2.05f}, {23.97f,38.14f,11.73f}, {11.15f,36.83f,13.93f}, {7.38f,16.73f,22.37f}, {25.39f,22.39f,25.64f}, {31.71f,5.72f,19.94f}, {2.49f,6.59f,5.49f}, {30.02f,2.34f,25.86f}, {7.13f,1.86f,7.11f},  },
{ {13.44f,71.49f,16.39f}, {3.73f,56.62f,11.65f}, {14.09f,65.03f,14.31f}, {17.66f,73.77f,24.44f}, {7.79f,71.66f,30.19f}, {-2.22f,30.78f,6.94f}, {3.19f,40.90f,6.18f}, {24.23f,40.54f,13.53f}, {11.33f,38.84f,14.42f}, {6.73f,17.36f,21.33f}, {25.51f,22.24f,26.63f}, {31.80f,5.72f,20.11f}, {2.75f,6.62f,4.70f}, {30.02f,2.33f,25.87f}, {7.21f,1.88f,6.97f},  },
{ {10.86f,76.32f,14.31f}, {2.18f,61.28f,8.88f}, {12.58f,69.92f,10.89f}, {17.21f,81.66f,17.13f}, {6.96f,82.49f,22.16f}, {-7.63f,37.09f,12.58f}, {-0.63f,45.47f,9.74f}, {22.22f,45.87f,15.83f}, {9.46f,43.31f,14.21f}, {4.85f,19.69f,18.43f}, {24.28f,23.12f,26.32f}, {31.61f,6.62f,20.52f}, {3.29f,6.33f,3.15f}, {29.91f,2.38f,25.92f}, {7.33f,1.93f,6.82f},  },
{ {9.90f,81.90f,13.01f}, {1.38f,68.56f,5.81f}, {13.24f,74.66f,7.61f}, {21.61f,85.93f,11.01f}, {11.99f,91.28f,13.13f}, {-18.97f,53.11f,8.75f}, {-8.25f,56.43f,7.26f}, {18.43f,51.10f,17.30f}, {5.84f,49.19f,13.67f}, {2.84f,22.69f,14.98f}, {20.31f,26.56f,26.00f}, {29.16f,10.04f,22.62f}, {2.59f,5.98f,2.89f}, {29.64f,3.54f,27.16f}, {7.10f,1.96f,6.93f},  },
{ {11.78f,84.72f,12.55f}, {1.63f,74.99f,3.82f}, {14.21f,76.74f,6.65f}, {26.52f,83.13f,13.08f}, {20.56f,91.84f,9.86f}, {-23.20f,75.92f,1.38f}, {-13.00f,70.70f,2.28f}, {15.11f,53.79f,17.24f}, {2.19f,52.69f,14.33f}, {0.61f,24.26f,11.62f}, {16.50f,28.41f,24.85f}, {23.22f,10.54f,23.77f}, {2.04f,5.60f,2.56f}, {22.82f,3.99f,28.22f}, {6.90f,1.98f,6.80f},  },
{ {14.76f,82.80f,12.46f}, {2.55f,78.01f,3.94f}, {14.78f,75.32f,7.59f}, {25.53f,72.72f,18.61f}, {25.99f,82.92f,15.12f}, {-13.90f,93.99f,1.49f}, {-11.71f,82.45f,0.40f}, {11.07f,54.84f,16.86f}, {-1.76f,54.63f,13.43f}, {-1.10f,24.69f,9.33f}, {12.71f,28.60f,23.36f}, {15.75f,9.77f,23.33f}, {2.22f,5.43f,2.03f}, {14.23f,3.77f,28.20f}, {6.91f,2.00f,6.66f},  },
{ {16.45f,77.44f,12.74f}, {3.78f,78.00f,5.26f}, {14.89f,70.93f,9.27f}, {18.13f,59.18f,19.67f}, {22.90f,68.23f,23.67f}, {-0.24f,97.28f,8.24f}, {-6.56f,88.79f,2.71f}, {7.46f,54.55f,16.29f}, {-4.95f,55.28f,12.04f}, {-2.45f,24.12f,7.67f}, {8.20f,28.06f,22.58f}, {8.41f,8.94f,24.22f}, {2.95f,5.01f,1.24f}, {5.64f,3.62f,29.42f}, {7.00f,2.02f,6.58f},  },
{ {16.30f,72.63f,13.92f}, {4.79f,76.02f,7.29f}, {12.90f,66.38f,10.37f}, {11.24f,50.73f,15.68f}, {12.29f,53.71f,26.43f}, {7.80f,89.97f,16.09f}, {-0.83f,89.39f,8.32f}, {5.19f,52.41f,15.84f}, {-7.12f,54.55f,11.98f}, {-4.13f,22.74f,6.87f}, {2.93f,26.79f,23.37f}, {-0.03f,7.89f,26.26f}, {3.55f,4.39f,0.51f}, {-5.17f,3.68f,31.12f}, {6.90f,1.99f,6.49f},  },
{ {15.13f,69.74f,16.14f}, {3.93f,73.59f,9.77f}, {11.60f,63.34f,11.57f}, {7.89f,47.00f,14.56f}, {6.24f,46.99f,25.47f}, {7.43f,81.81f,21.14f}, {-0.56f,86.24f,14.15f}, {3.32f,49.52f,16.22f}, {-8.93f,52.57f,12.84f}, {-5.91f,21.15f,6.97f}, {-3.12f,25.34f,24.53f}, {-8.12f,6.81f,26.47f}, {4.19f,4.16f,0.18f}, {-14.17f,2.78f,30.57f}, {6.70f,1.97f,6.55f},  },
{ {12.62f,69.22f,19.14f}, {1.46f,71.98f,11.82f}, {9.52f,62.16f,14.75f}, {6.60f,46.24f,18.91f}, {7.61f,47.96f,29.67f}, {0.65f,80.72f,23.11f}, {-5.77f,84.47f,14.35f}, {-0.34f,47.94f,19.26f}, {-11.63f,50.79f,13.52f}, {-7.49f,19.89f,6.93f}, {-9.05f,24.54f,26.54f}, {-10.01f,5.50f,25.34f}, {4.59f,4.42f,0.05f}, {-15.31f,2.47f,29.76f}, {6.18f,2.03f,6.68f},  },
{ {7.41f,73.03f,22.16f}, {-2.43f,72.22f,12.33f}, {6.49f,64.37f,18.36f}, {6.46f,52.06f,28.26f}, {9.14f,60.16f,35.00f}, {-9.35f,85.88f,18.33f}, {-11.90f,83.25f,7.36f}, {-3.92f,48.43f,22.63f}, {-13.61f,49.76f,14.03f}, {-6.80f,19.85f,6.82f}, {-12.24f,24.13f,28.16f}, {-10.65f,5.31f,25.41f}, {6.10f,4.84f,0.38f}, {-15.57f,2.52f,29.47f}, {5.51f,2.06f,6.84f},  },
{ {-1.28f,79.77f,22.92f}, {-5.33f,73.38f,10.96f}, {3.15f,70.31f,20.32f}, {5.69f,67.64f,35.28f}, {5.30f,78.39f,33.61f}, {-16.31f,86.46f,4.43f}, {-14.58f,77.37f,-1.73f}, {-7.69f,50.17f,25.56f}, {-13.88f,49.85f,14.04f}, {-4.75f,22.16f,6.53f}, {-13.20f,24.03f,28.87f}, {-10.43f,5.32f,26.12f}, {7.60f,6.07f,1.79f}, {-15.62f,2.59f,29.52f}, {4.85f,2.17f,7.05f},  },
{ {-8.62f,81.91f,19.46f}, {-6.11f,72.29f,8.29f}, {-0.18f,75.30f,19.78f}, {2.06f,82.90f,32.77f}, {-4.35f,88.61f,25.89f}, {-18.12f,73.99f,-6.21f}, {-13.59f,64.52f,-3.40f}, {-11.04f,51.85f,27.28f}, {-12.74f,51.25f,14.31f}, {-1.13f,24.97f,7.42f}, {-14.18f,24.55f,29.37f}, {-10.02f,5.89f,28.63f}, {9.72f,7.07f,5.17f}, {-15.91f,2.58f,29.98f}, {4.99f,2.27f,7.78f},  },
{ {-12.47f,81.70f,16.68f}, {-5.10f,69.67f,7.13f}, {-3.68f,77.42f,18.80f}, {-5.59f,88.99f,27.36f}, {-13.55f,87.24f,19.58f}, {-16.42f,57.53f,-4.47f}, {-7.19f,54.86f,1.71f}, {-12.64f,53.87f,26.66f}, {-11.46f,51.83f,13.82f}, {1.63f,26.12f,9.58f}, {-12.98f,24.95f,29.72f}, {-10.08f,6.09f,30.05f}, {10.72f,7.22f,8.72f}, {-16.23f,2.55f,30.22f}, {5.21f,2.18f,8.40f},  },
{ {-17.16f,82.04f,14.03f}, {-7.52f,70.78f,5.96f}, {-9.64f,78.54f,17.55f}, {-16.57f,86.38f,28.95f}, {-24.31f,81.90f,22.07f}, {-11.84f,47.50f,0.78f}, {-2.96f,55.35f,2.58f}, {-11.77f,53.74f,25.86f}, {-8.34f,51.94f,13.39f}, {3.43f,26.98f,9.79f}, {-10.88f,26.11f,30.31f}, {-10.48f,7.03f,30.97f}, {10.37f,7.49f,9.01f}, {-16.56f,2.59f,30.29f}, {5.21f,2.05f,8.54f},  },
{ {-22.13f,79.43f,12.49f}, {-12.22f,68.41f,4.43f}, {-17.09f,75.09f,17.26f}, {-20.98f,75.67f,32.92f}, {-24.03f,64.83f,30.35f}, {-1.34f,47.99f,6.73f}, {0.87f,58.48f,1.97f}, {-9.66f,52.25f,26.02f}, {-5.48f,50.74f,13.56f}, {5.08f,27.64f,9.96f}, {-8.88f,26.71f,30.75f}, {-10.98f,7.75f,31.07f}, {10.00f,7.79f,9.05f}, {-16.81f,2.63f,30.27f}, {5.09f,2.10f,8.68f},  },
{ {-25.81f,72.95f,12.27f}, {-14.80f,63.18f,4.14f}, {-21.22f,66.72f,18.48f}, {-15.88f,66.37f,34.59f}, {-9.86f,56.71f,32.31f}, {4.70f,55.90f,9.89f}, {1.36f,63.51f,1.22f}, {-7.72f,50.00f,26.36f}, {-2.95f,49.23f,13.90f}, {4.35f,27.17f,9.22f}, {-8.41f,26.43f,30.81f}, {-10.87f,7.55f,31.11f}, {10.27f,7.52f,8.53f}, {-16.79f,2.61f,30.31f}, {4.97f,2.18f,8.68f},  },
{ {-27.52f,65.61f,13.47f}, {-15.83f,57.75f,4.74f}, {-21.01f,59.93f,20.12f}, {-10.85f,63.38f,33.59f}, {-1.58f,57.50f,28.78f}, {5.04f,59.98f,9.29f}, {-1.57f,64.10f,-0.24f}, {-5.39f,47.90f,25.89f}, {-0.80f,46.98f,13.34f}, {3.84f,25.66f,8.39f}, {-9.34f,26.39f,30.35f}, {-10.59f,7.26f,31.22f}, {11.45f,6.42f,8.53f}, {-16.67f,2.56f,30.50f}, {5.30f,2.14f,8.60f},  },
{ {-26.65f,59.38f,14.87f}, {-15.02f,52.44f,5.15f}, {-18.77f,54.39f,20.72f}, {-8.15f,59.20f,33.43f}, {2.92f,55.92f,29.00f}, {7.70f,56.36f,6.06f}, {-1.25f,57.50f,-2.56f}, {-2.74f,45.36f,24.97f}, {0.90f,43.79f,12.26f}, {2.98f,22.83f,7.42f}, {-10.30f,26.09f,28.84f}, {-10.51f,7.06f,30.89f}, {12.27f,4.78f,8.93f}, {-16.70f,2.58f,30.45f}, {5.67f,2.06f,8.58f},  },
{ {-24.32f,55.62f,14.82f}, {-13.13f,48.31f,4.92f}, {-15.91f,50.69f,19.96f}, {-4.54f,52.38f,33.11f}, {7.18f,48.42f,32.25f}, {12.59f,47.60f,-0.51f}, {1.05f,48.24f,-4.92f}, {0.67f,43.31f,23.30f}, {3.25f,41.51f,10.33f}, {2.03f,21.54f,6.64f}, {-7.76f,25.25f,27.30f}, {-10.02f,6.56f,30.55f}, {12.21f,4.25f,9.31f}, {-16.71f,2.60f,30.38f}, {5.65f,2.01f,8.76f},  },
{ {-21.33f,53.77f,14.15f}, {-10.59f,46.27f,4.69f}, {-12.81f,49.14f,18.04f}, {-3.01f,44.04f,32.77f}, {5.68f,35.98f,35.56f}, {11.35f,33.34f,-8.06f}, {0.91f,39.46f,-6.78f}, {5.08f,43.79f,21.69f}, {6.33f,41.06f,8.93f}, {4.04f,21.58f,5.50f}, {-2.40f,25.79f,26.43f}, {-7.49f,7.55f,30.36f}, {12.33f,3.68f,9.38f}, {-14.21f,2.85f,29.53f}, {5.73f,1.99f,8.77f},  },
{ {-17.58f,52.83f,13.10f}, {-7.97f,45.53f,5.22f}, {-9.20f,47.91f,15.74f}, {-6.62f,37.32f,28.72f}, {-5.32f,25.71f,32.97f}, {-1.26f,22.40f,-9.20f}, {-3.80f,33.98f,-6.02f}, {9.32f,43.98f,20.98f}, {9.69f,40.60f,8.32f}, {5.40f,22.13f,5.30f}, {1.05f,26.46f,23.79f}, {8.16f,9.18f,28.36f}, {12.40f,3.58f,9.31f}, {3.65f,2.70f,27.95f}, {5.72f,2.05f,8.90f},  },
{ {-14.05f,52.90f,12.28f}, {-5.45f,46.48f,5.44f}, {-6.50f,48.40f,14.24f}, {-11.03f,36.43f,22.49f}, {-16.97f,25.82f,23.23f}, {-14.67f,23.34f,-2.17f}, {-9.17f,34.72f,-1.87f}, {12.66f,42.89f,20.66f}, {12.55f,40.24f,7.78f}, {6.81f,22.37f,4.71f}, {8.05f,22.26f,22.70f}, {23.80f,11.88f,25.93f}, {12.48f,3.44f,9.12f}, {25.57f,4.02f,24.47f}, {5.76f,2.03f,8.85f},  },
{ {-10.78f,55.12f,12.35f}, {-2.72f,49.55f,5.94f}, {-3.67f,50.51f,13.67f}, {-11.46f,39.53f,18.68f}, {-20.03f,31.25f,16.20f}, {-19.35f,31.36f,3.29f}, {-9.79f,39.25f,0.80f}, {15.37f,41.84f,20.47f}, {14.43f,39.95f,7.64f}, {7.36f,22.42f,4.31f}, {17.57f,18.19f,22.18f}, {35.00f,10.26f,23.36f}, {12.50f,3.37f,8.95f}, {37.08f,2.77f,21.37f}, {5.79f,2.02f,8.75f},  },
{ {-6.38f,59.45f,13.48f}, {1.43f,53.12f,6.19f}, {0.75f,55.38f,14.94f}, {-6.15f,44.36f,23.42f}, {-14.63f,37.56f,18.47f}, {-15.86f,37.84f,3.28f}, {-5.13f,42.18f,0.19f}, {18.09f,41.36f,20.81f}, {15.33f,39.38f,8.25f}, {6.48f,22.45f,4.81f}, {23.79f,16.69f,22.39f}, {41.11f,8.89f,21.76f}, {12.42f,3.46f,8.90f}, {39.94f,1.98f,20.59f}, {5.76f,2.02f,8.72f},  },
{ {-2.12f,62.28f,14.04f}, {4.72f,53.40f,5.28f}, {5.74f,59.62f,15.11f}, {7.18f,56.69f,31.62f}, {-0.32f,49.68f,28.17f}, {-9.52f,38.55f,7.76f}, {1.38f,39.08f,5.60f}, {20.59f,42.19f,21.74f}, {16.30f,39.21f,9.78f}, {6.21f,23.18f,5.84f}, {24.74f,16.15f,22.86f}, {42.28f,8.47f,21.56f}, {12.15f,4.01f,8.77f}, {40.15f,2.06f,20.58f}, {5.56f,2.09f,8.81f},  },
{ {0.37f,64.54f,15.45f}, {5.50f,53.53f,8.46f}, {10.01f,63.22f,14.36f}, {14.29f,67.47f,29.24f}, {6.32f,60.47f,28.02f}, {-4.28f,38.33f,18.68f}, {4.45f,39.13f,12.51f}, {21.79f,42.75f,21.38f}, {17.22f,39.68f,9.79f}, {7.18f,23.13f,5.99f}, {26.81f,16.64f,24.60f}, {43.72f,7.90f,22.04f}, {12.17f,3.96f,9.37f}, {40.51f,2.06f,20.92f}, {5.57f,2.15f,9.26f},  },
{ {2.94f,69.57f,17.93f}, {7.44f,58.85f,9.60f}, {12.73f,67.36f,16.21f}, {6.16f,68.36f,29.28f}, {-0.36f,61.48f,23.14f}, {-0.39f,38.13f,14.48f}, {7.36f,43.26f,8.56f}, {23.42f,44.37f,21.44f}, {16.23f,42.33f,10.79f}, {6.09f,24.48f,6.78f}, {29.43f,19.05f,25.04f}, {44.57f,7.84f,21.44f}, {11.78f,5.15f,8.71f}, {40.71f,2.08f,20.97f}, {5.50f,2.35f,9.62f},  },
{ {3.86f,75.82f,19.97f}, {7.68f,66.75f,8.45f}, {12.45f,71.26f,18.80f}, {2.93f,67.36f,29.31f}, {-5.68f,65.08f,22.09f}, {-4.34f,49.22f,-4.24f}, {5.50f,54.10f,-1.81f}, {20.01f,47.09f,20.74f}, {12.84f,47.37f,10.21f}, {5.22f,24.82f,6.54f}, {30.34f,23.09f,22.58f}, {43.61f,8.99f,21.12f}, {11.43f,5.55f,7.87f}, {40.51f,2.03f,20.83f}, {5.47f,2.18f,9.25f},  },
{ {3.48f,81.16f,19.76f}, {8.50f,75.29f,8.55f}, {11.66f,73.92f,20.71f}, {6.60f,62.85f,32.05f}, {-4.73f,63.96f,31.60f}, {-7.36f,76.88f,-10.78f}, {2.53f,73.78f,-6.35f}, {14.34f,50.61f,19.71f}, {8.63f,50.75f,8.19f}, {6.01f,25.41f,4.76f}, {27.59f,27.19f,21.75f}, {38.63f,11.34f,21.55f}, {11.20f,5.83f,7.52f}, {38.49f,3.00f,20.97f}, {5.34f,2.07f,9.05f},  },
{ {4.20f,83.89f,20.80f}, {9.14f,79.64f,11.10f}, {11.91f,73.74f,21.46f}, {9.45f,57.04f,25.34f}, {-1.82f,55.73f,23.86f}, {-7.42f,94.82f,1.51f}, {1.49f,88.86f,1.29f}, {9.56f,53.43f,18.13f}, {4.53f,54.98f,6.19f}, {6.46f,26.00f,4.02f}, {21.64f,29.23f,21.75f}, {30.25f,11.98f,22.78f}, {10.67f,6.23f,7.29f}, {29.47f,3.83f,24.11f}, {5.15f,2.07f,9.05f},  },
{ {5.83f,83.88f,22.73f}, {9.14f,80.25f,13.18f}, {12.03f,71.45f,20.67f}, {11.96f,53.44f,20.02f}, {0.38f,50.42f,19.76f}, {-7.66f,94.47f,1.27f}, {2.08f,92.75f,10.41f}, {7.07f,55.08f,16.92f}, {1.77f,57.54f,5.28f}, {6.57f,26.34f,2.83f}, {13.68f,29.19f,22.27f}, {21.43f,11.62f,23.59f}, {10.21f,6.48f,6.57f}, {19.80f,3.92f,25.78f}, {5.00f,2.04f,8.85f},  },
{ {6.98f,82.95f,23.98f}, {7.87f,79.34f,14.45f}, {13.80f,70.52f,20.61f}, {15.93f,52.85f,21.88f}, {6.48f,48.07f,27.15f}, {-8.10f,93.46f,0.51f}, {-0.08f,90.91f,11.48f}, {5.28f,53.94f,17.09f}, {-0.38f,57.10f,5.80f}, {5.81f,26.25f,1.81f}, {6.12f,28.11f,21.72f}, {14.02f,10.71f,22.70f}, {9.85f,6.43f,5.69f}, {11.25f,4.02f,25.70f}, {5.00f,1.98f,8.65f},  },
{ {7.25f,82.00f,26.45f}, {6.09f,78.22f,16.15f}, {14.67f,70.06f,21.36f}, {22.31f,55.38f,24.10f}, {19.46f,51.68f,34.90f}, {-8.51f,91.69f,-0.94f}, {-2.94f,87.82f,7.99f}, {4.90f,52.33f,18.05f}, {-2.02f,55.89f,7.63f}, {4.64f,26.03f,2.08f}, {0.85f,27.95f,21.34f}, {6.07f,9.47f,21.93f}, {9.84f,6.40f,5.63f}, {1.33f,3.90f,24.45f}, {5.00f,2.00f,8.67f},  },
{ {5.61f,80.55f,29.66f}, {3.32f,77.04f,18.12f}, {13.51f,70.04f,23.23f}, {26.34f,59.89f,22.38f}, {29.32f,58.04f,33.45f}, {-6.68f,88.18f,-4.41f}, {-0.44f,82.70f,3.65f}, {3.12f,50.63f,19.23f}, {-3.54f,54.76f,9.02f}, {3.51f,25.42f,2.70f}, {-2.36f,27.48f,22.77f}, {-2.66f,8.45f,25.81f}, {10.23f,6.14f,5.98f}, {-8.97f,3.83f,27.69f}, {5.10f,2.01f,8.70f},  },
{ {2.42f,79.00f,31.42f}, {1.82f,75.39f,19.00f}, {11.24f,68.98f,25.23f}, {24.15f,60.57f,21.55f}, {29.48f,58.58f,31.63f}, {2.39f,81.63f,-7.37f}, {3.95f,78.11f,3.66f}, {0.63f,48.44f,21.17f}, {-4.58f,53.09f,10.37f}, {2.76f,24.13f,3.55f}, {-6.73f,26.51f,25.51f}, {-5.92f,7.92f,30.46f}, {10.91f,5.23f,6.00f}, {-12.32f,2.80f,30.50f}, {5.33f,2.02f,8.67f},  },
{ {-0.42f,79.01f,31.41f}, {1.41f,74.29f,18.65f}, {8.49f,67.58f,28.28f}, {19.79f,56.02f,32.24f}, {20.54f,55.02f,43.63f}, {3.28f,79.79f,-7.39f}, {5.24f,75.58f,3.36f}, {-1.80f,47.41f,23.36f}, {-4.95f,51.79f,11.41f}, {0.71f,22.79f,5.04f}, {-7.91f,24.84f,27.65f}, {-5.37f,6.44f,32.67f}, {11.34f,5.17f,6.98f}, {-11.87f,2.48f,31.21f}, {5.36f,2.06f,8.69f},  },
{ {-2.32f,79.34f,30.08f}, {1.38f,72.82f,17.84f}, {4.52f,67.34f,31.37f}, {5.55f,56.44f,44.33f}, {-2.74f,57.94f,51.94f}, {-4.46f,79.21f,-5.14f}, {3.61f,74.11f,2.20f}, {-1.62f,46.71f,25.61f}, {-3.36f,49.57f,12.90f}, {-1.49f,21.95f,6.33f}, {-9.54f,24.14f,28.59f}, {-5.63f,5.71f,33.16f}, {11.13f,5.94f,8.38f}, {-11.86f,2.53f,31.26f}, {5.28f,2.23f,9.01f},  },
{ {-5.35f,78.41f,27.86f}, {-1.96f,70.35f,17.30f}, {-1.88f,68.71f,30.43f}, {-7.02f,62.91f,44.43f}, {-17.39f,62.74f,39.30f}, {-12.93f,72.79f,2.71f}, {-1.36f,71.81f,1.44f}, {0.82f,45.39f,28.01f}, {-0.07f,46.10f,15.00f}, {-0.89f,21.98f,7.29f}, {-8.94f,24.17f,29.48f}, {-5.63f,5.65f,33.32f}, {11.70f,6.07f,9.22f}, {-11.74f,2.59f,31.07f}, {5.66f,2.27f,9.09f},  },
{ {-8.42f,75.26f,24.39f}, {-5.03f,65.21f,14.80f}, {-5.71f,67.14f,27.31f}, {-7.41f,61.32f,42.94f}, {-12.59f,52.98f,36.43f}, {-12.69f,57.37f,7.31f}, {-6.04f,61.79f,-0.92f}, {1.54f,44.61f,29.68f}, {2.59f,43.80f,16.75f}, {1.32f,22.62f,7.76f}, {-8.26f,23.99f,28.48f}, {-6.31f,5.75f,33.98f}, {12.39f,5.53f,9.81f}, {-11.83f,2.54f,30.60f}, {6.17f,2.20f,9.09f},  },
{ {-9.12f,73.00f,20.84f}, {-5.56f,60.18f,10.92f}, {-4.68f,65.70f,24.82f}, {2.86f,61.37f,40.55f}, {2.35f,49.84f,42.40f}, {-11.38f,44.91f,9.92f}, {-4.61f,46.85f,1.02f}, {2.12f,44.83f,29.98f}, {3.97f,42.39f,17.35f}, {3.65f,22.22f,7.93f}, {-6.99f,23.93f,28.54f}, {-7.61f,5.86f,35.24f}, {12.93f,4.34f,10.86f}, {-12.50f,2.50f,30.81f}, {6.43f,2.12f,9.22f},  },
{ {-6.81f,74.23f,19.39f}, {-4.30f,60.33f,9.81f}, {0.42f,69.56f,21.20f}, {13.15f,69.71f,33.78f}, {13.70f,61.80f,42.18f}, {-11.95f,42.61f,19.73f}, {-4.03f,43.27f,11.39f}, {5.37f,47.66f,29.08f}, {4.69f,44.03f,16.55f}, {4.87f,22.39f,7.64f}, {-1.65f,24.69f,32.87f}, {-6.14f,6.09f,36.08f}, {13.00f,3.99f,10.91f}, {-11.88f,2.99f,31.24f}, {6.61f,2.10f,8.86f},  },
{ {-0.56f,77.81f,20.18f}, {-3.81f,63.93f,11.49f}, {6.11f,73.34f,17.73f}, {21.12f,75.09f,25.51f}, {23.01f,77.15f,36.81f}, {-16.18f,53.59f,28.29f}, {-9.48f,51.24f,19.42f}, {9.15f,50.24f,29.04f}, {4.25f,46.08f,17.69f}, {5.98f,22.90f,8.06f}, {4.57f,25.62f,34.24f}, {8.93f,6.94f,35.20f}, {13.01f,3.88f,10.85f}, {2.79f,2.90f,37.01f}, {6.64f,2.07f,8.68f},  },
{ {6.47f,82.56f,20.45f}, {-1.95f,69.81f,14.94f}, {10.15f,76.27f,16.72f}, {27.28f,77.46f,17.71f}, {31.91f,85.85f,23.68f}, {-17.69f,70.58f,29.80f}, {-11.66f,62.36f,24.18f}, {12.96f,53.30f,29.58f}, {3.14f,50.43f,21.67f}, {6.10f,24.48f,11.40f}, {14.91f,26.30f,34.24f}, {18.82f,8.78f,28.12f}, {12.69f,4.88f,9.97f}, {19.33f,4.07f,33.21f}, {6.37f,2.14f,8.73f},  },
{ {11.06f,84.78f,18.04f}, {-0.55f,74.02f,20.24f}, {11.80f,76.66f,15.53f}, {25.95f,73.62f,6.09f}, {32.43f,82.45f,8.45f}, {-12.73f,84.34f,28.23f}, {-11.98f,73.04f,29.34f}, {19.97f,55.01f,28.57f}, {7.13f,53.60f,29.52f}, {2.07f,26.81f,19.76f}, {21.59f,26.31f,29.11f}, {19.79f,7.51f,26.24f}, {8.41f,10.62f,8.67f}, {20.28f,2.54f,31.76f}, {6.03f,3.78f,9.56f},  },
{ {11.04f,85.31f,16.25f}, {3.21f,77.69f,25.21f}, {10.10f,76.30f,14.92f}, {14.67f,65.86f,1.67f}, {24.30f,71.19f,-0.91f}, {-3.68f,91.35f,28.45f}, {-4.80f,83.04f,35.65f}, {22.39f,55.30f,23.29f}, {12.63f,55.36f,32.14f}, {6.90f,27.82f,33.67f}, {22.77f,26.53f,25.43f}, {20.56f,7.73f,25.98f}, {5.58f,21.21f,14.28f}, {20.70f,2.53f,31.79f}, {7.52f,15.57f,9.92f},  },
};