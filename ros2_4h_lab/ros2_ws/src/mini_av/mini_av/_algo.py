"""노드가 쓸 알고리즘 모듈 고르기: use_solution 파라미터에 따라 정답 / 내 코드"""


def load(use_solution):
    if use_solution:
        from mini_av import solutions as m
    else:
        from mini_av import my_algorithms as m
    return m
