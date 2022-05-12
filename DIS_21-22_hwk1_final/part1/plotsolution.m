function plotsolution(cities,solution,text)

if(nargin<3) text='t';
end;
hold on;
plotcities(cities);

for I=1:length(solution)-1,
    plot([cities(solution(I),1) cities(solution(I+1),1)],[cities(solution(I),2) cities(solution(I+1),2)]);
end;

plot([cities(solution(1),1) cities(solution(length(solution)),1)],[cities(solution(1),2) cities(solution(length(solution)),2)],'--');

title(sprintf('%s',text));