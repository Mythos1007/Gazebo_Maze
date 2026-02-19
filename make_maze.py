#!/usr/bin/env python3
"""
Gazebo 미로 생성 프로그램
- 중앙 좌표 0,0 기준
- 1m x 1m 정사각형 셀
- 외곽 테두리 고정, 내부 벽 생성
- 미로 크기 및 최소 벽 개수 설정 가능
- DFS 알고리즘으로 모든 경로가 연결됨을 보장
"""

import random
import argparse
import os
from collections import deque


class MazeGenerator:
    def __init__(self, grid_size=5, cell_size=1.0, min_walls=5, output_file="maze.sdf"):
        """
        미로 생성기 초기화

        Args:
            grid_size: 미로 그리드 크기 (NxN)
            cell_size: 각 셀의 크기 (미터)
            min_walls: 내부에 생성될 최소 벽 개수
            output_file: 출력 SDF 파일 이름
        """
        self.grid_size = grid_size
        self.cell_size = cell_size
        self.min_walls = min_walls
        self.output_file = output_file
        self.wall_height = 0.5
        self.wall_thickness = 0.05

        # 셀 간의 벽 정보 (True = 벽 있음, False = 벽 없음)
        # horizontal_walls[row][col] = row와 row+1 사이의 벽 (가로 방향 벽)
        # vertical_walls[row][col] = col와 col+1 사이의 벽 (세로 방향 벽)
        self.horizontal_walls = [[True] * grid_size for _ in range(grid_size + 1)]
        self.vertical_walls = [[True] * (grid_size + 1) for _ in range(grid_size)]

    def generate_maze(self):
        """미로 생성 - DFS 알고리즘으로 연결성 보장"""
        # 1. DFS로 연결된 미로 생성
        self._generate_connected_maze_dfs()

        # 2. 현재 내부 벽 개수 확인
        current_inner_walls = self._count_inner_walls()
        print(f"DFS 생성 후 내부 벽 개수: {current_inner_walls}")

        # 3. 최소 벽 개수를 맞추기 위해 추가 벽 배치 (연결성 유지)
        if current_inner_walls < self.min_walls:
            self._add_additional_walls(self.min_walls - current_inner_walls)

        # 4. 벽 정보를 3D 좌표로 변환
        walls = self._convert_to_3d_walls()

        return walls

    def _generate_connected_maze_dfs(self):
        """DFS 알고리즘으로 모든 셀이 연결된 미로 생성"""
        # 모든 셀을 미방문 상태로 초기화
        visited = [[False] * self.grid_size for _ in range(self.grid_size)]

        # 랜덤 시작 위치에서 DFS 시작
        start_row = random.randint(0, self.grid_size - 1)
        start_col = random.randint(0, self.grid_size - 1)

        self._dfs_carve(start_row, start_col, visited)

    def _dfs_carve(self, row, col, visited):
        """DFS로 경로를 깎아내며 미로 생성"""
        visited[row][col] = True

        # 4방향을 랜덤 순서로 탐색
        directions = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # 우, 하, 좌, 상
        random.shuffle(directions)

        for dr, dc in directions:
            next_row, next_col = row + dr, col + dc

            # 유효한 범위이고 아직 방문하지 않은 셀인 경우
            if (0 <= next_row < self.grid_size and
                0 <= next_col < self.grid_size and
                not visited[next_row][next_col]):

                # 현재 셀과 다음 셀 사이의 벽을 제거
                if dr == 1:  # 아래로 이동
                    self.horizontal_walls[row + 1][col] = False
                elif dr == -1:  # 위로 이동
                    self.horizontal_walls[row][col] = False
                elif dc == 1:  # 오른쪽으로 이동
                    self.vertical_walls[row][col + 1] = False
                elif dc == -1:  # 왼쪽으로 이동
                    self.vertical_walls[row][col] = False

                # 재귀적으로 다음 셀 탐색
                self._dfs_carve(next_row, next_col, visited)

    def _count_inner_walls(self):
        """내부 벽 개수 카운트 (외곽 제외)"""
        count = 0

        # 내부 수평 벽
        for row in range(1, self.grid_size):
            for col in range(self.grid_size):
                if self.horizontal_walls[row][col]:
                    count += 1

        # 내부 수직 벽
        for row in range(self.grid_size):
            for col in range(1, self.grid_size):
                if self.vertical_walls[row][col]:
                    count += 1

        return count

    def _add_additional_walls(self, num_walls_to_add):
        """연결성을 해치지 않는 선에서 추가 벽 배치"""
        # 제거할 수 있는 벽 목록 (현재 열려있는 내부 벽)
        removable_paths = []

        # 내부 수평 경로
        for row in range(1, self.grid_size):
            for col in range(self.grid_size):
                if not self.horizontal_walls[row][col]:
                    removable_paths.append(('h', row, col))

        # 내부 수직 경로
        for row in range(self.grid_size):
            for col in range(1, self.grid_size):
                if not self.vertical_walls[row][col]:
                    removable_paths.append(('v', row, col))

        # 랜덤하게 섞기
        random.shuffle(removable_paths)

        # 벽 추가 (연결성 확인하면서)
        added = 0
        for path_type, row, col in removable_paths:
            if added >= num_walls_to_add:
                break

            # 임시로 벽을 추가해보기
            if path_type == 'h':
                self.horizontal_walls[row][col] = True
            else:
                self.vertical_walls[row][col] = True

            # 연결성 확인
            if self._is_all_connected():
                added += 1
                print(f"추가 벽 배치: {added}/{num_walls_to_add}")
            else:
                # 연결성이 깨지면 다시 제거
                if path_type == 'h':
                    self.horizontal_walls[row][col] = False
                else:
                    self.vertical_walls[row][col] = False

        if added < num_walls_to_add:
            print(f"경고: 연결성을 유지하면서 {added}개의 벽만 추가 가능했습니다.")

    def _is_all_connected(self):
        """모든 셀이 연결되어 있는지 BFS로 확인"""
        visited = [[False] * self.grid_size for _ in range(self.grid_size)]
        queue = deque([(0, 0)])
        visited[0][0] = True
        count = 1

        while queue:
            row, col = queue.popleft()

            # 4방향 확인
            # 위
            if row > 0 and not self.horizontal_walls[row][col] and not visited[row - 1][col]:
                visited[row - 1][col] = True
                queue.append((row - 1, col))
                count += 1
            # 아래
            if row < self.grid_size - 1 and not self.horizontal_walls[row + 1][col] and not visited[row + 1][col]:
                visited[row + 1][col] = True
                queue.append((row + 1, col))
                count += 1
            # 왼쪽
            if col > 0 and not self.vertical_walls[row][col] and not visited[row][col - 1]:
                visited[row][col - 1] = True
                queue.append((row, col - 1))
                count += 1
            # 오른쪽
            if col < self.grid_size - 1 and not self.vertical_walls[row][col + 1] and not visited[row][col + 1]:
                visited[row][col + 1] = True
                queue.append((row, col + 1))
                count += 1

        return count == self.grid_size * self.grid_size

    def _convert_to_3d_walls(self):
        """벽 정보를 3D 좌표로 변환"""
        walls = []
        half_size = (self.grid_size * self.cell_size) / 2.0

        # 수평 벽들
        for row in range(self.grid_size + 1):
            for col in range(self.grid_size):
                if self.horizontal_walls[row][col]:
                    # 벽의 중심 좌표 계산
                    x = -half_size + (col + 0.5) * self.cell_size
                    y = -half_size + row * self.cell_size
                    walls.append(self._create_wall(x, y, 0, "horizontal"))

        # 수직 벽들
        for row in range(self.grid_size):
            for col in range(self.grid_size + 1):
                if self.vertical_walls[row][col]:
                    # 벽의 중심 좌표 계산
                    x = -half_size + col * self.cell_size
                    y = -half_size + (row + 0.5) * self.cell_size
                    walls.append(self._create_wall(x, y, 0, "vertical"))

        return walls

    def _create_wall(self, x, y, z, orientation):
        """
        벽 객체 생성

        Args:
            x, y, z: 벽의 중심 좌표
            orientation: 'horizontal' 또는 'vertical'
        """
        if orientation == "horizontal":
            length = self.cell_size
            width = self.wall_thickness
            yaw = 0
        else:  # vertical
            length = self.wall_thickness
            width = self.cell_size
            yaw = 0

        return {
            'x': x,
            'y': y,
            'z': z + self.wall_height / 2.0,
            'length': length,
            'width': width,
            'height': self.wall_height,
            'yaw': yaw
        }

    def save_to_sdf(self, walls):
        """SDF 파일로 저장"""
        sdf_content = self._generate_sdf_content(walls)

        with open(self.output_file, 'w') as f:
            f.write(sdf_content)

        print(f"\n미로가 '{self.output_file}' 파일로 생성되었습니다.")
        print(f"총 벽 개수: {len(walls)}")

        # 외곽 벽 개수
        outer_walls = 2 * self.grid_size + 2 * self.grid_size
        inner_walls = len(walls) - outer_walls
        print(f"  - 외곽 벽: {outer_walls}개")
        print(f"  - 내부 벽: {inner_walls}개")
        print(f"미로 크기: {self.grid_size}x{self.grid_size} ({self.grid_size * self.cell_size}m x {self.grid_size * self.cell_size}m)")
        print(f"모든 영역이 연결되어 있습니다. ✓")

    def _generate_sdf_content(self, walls):
        """SDF 파일 내용 생성"""
        sdf_header = """<?xml version="1.0" ?>
<sdf version="1.6">
  <world name="maze_world">

    <!-- 조명 -->
    <include>
      <uri>model://sun</uri>
    </include>

    <!-- 바닥 -->
    <include>
      <uri>model://ground_plane</uri>
    </include>

    <!-- 미로 벽들 -->
    <model name="maze">
      <static>true</static>
      <pose>0 0 0 0 0 0</pose>
"""

        sdf_footer = """    </model>
  </world>
</sdf>
"""

        # 각 벽 생성
        wall_links = ""
        for idx, wall in enumerate(walls):
            wall_links += f"""
      <link name="wall_{idx}">
        <pose>{wall['x']} {wall['y']} {wall['z']} 0 0 {wall['yaw']}</pose>
        <collision name="collision">
          <geometry>
            <box>
              <size>{wall['length']} {wall['width']} {wall['height']}</size>
            </box>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <box>
              <size>{wall['length']} {wall['width']} {wall['height']}</size>
            </box>
          </geometry>
          <material>
            <ambient>0.7 0.7 0.7 1</ambient>
            <diffuse>0.7 0.7 0.7 1</diffuse>
            <specular>0.1 0.1 0.1 1</specular>
          </material>
        </visual>
      </link>
"""

        return sdf_header + wall_links + sdf_footer


def main():
    parser = argparse.ArgumentParser(description='Gazebo 미로 생성 프로그램')
    parser.add_argument('-s', '--size', type=int, default=5,
                        help='미로 그리드 크기 (기본값: 5)')
    parser.add_argument('-c', '--cell-size', type=float, default=1.0,
                        help='각 셀의 크기 (미터, 기본값: 1.0)')
    parser.add_argument('-m', '--min-walls', type=int, default=5,
                        help='최소 내부 벽 개수 (기본값: 5)')
    parser.add_argument('-o', '--output', type=str, default='maze.sdf',
                        help='출력 파일 이름 (기본값: maze.sdf)')
    parser.add_argument('--seed', type=int, default=None,
                        help='랜덤 시드 (재현 가능한 미로 생성)')

    args = parser.parse_args()

    # 랜덤 시드 설정
    if args.seed is not None:
        random.seed(args.seed)
        print(f"랜덤 시드: {args.seed}")

    # 미로 생성
    print(f"\n=== 미로 생성 시작 ===")
    print(f"그리드 크기: {args.size}x{args.size}")
    print(f"셀 크기: {args.cell_size}m")
    print(f"최소 내부 벽: {args.min_walls}개")
    print(f"출력 파일: {args.output}\n")

    generator = MazeGenerator(
        grid_size=args.size,
        cell_size=args.cell_size,
        min_walls=args.min_walls,
        output_file=args.output
    )

    walls = generator.generate_maze()
    generator.save_to_sdf(walls)

    print(f"\n=== 미로 생성 완료 ===")
    print(f"\nGazebo에서 실행하려면:")
    print(f"  gazebo {args.output}")


if __name__ == "__main__":
    main()
